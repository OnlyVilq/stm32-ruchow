#![no_std]
#![no_main]

use stm32g0xx_hal as hal;
use hal::{
    pac::{self, interrupt},
    prelude::*,
    serial::FullConfig,
};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use portable_atomic::{AtomicU32, Ordering};
use core::ptr::addr_of_mut;

use defmt_rtt as _;
use panic_probe as _;
use defmt::*;

//includowanie convertera
mod converter;
use converter::{
    converter,
    STEPS
};

// --- piny
// PA0	Silnik 2 (STEP)	TIM2_CH1 (AF2)	Wejście STEP (PUL) sterownika silnika nr 2
// PA7	Silnik 3 (STEP)	TIM3_CH2 (AF1)	Wejście STEP (PUL) sterownika silnika nr 3
// PA9	UART TX	USART1_TX (AF1)	Do RX konwertera USB-UART lub innego mikrokontrolera
// PA10	UART RX	USART1_RX (AF1)	Do TX konwertera USB-UART (tędy wchodzą dane DMA)
// PB0
// PB1


// --- ZMIENNE GLOBALNE ---
static G_TIM3: Mutex<RefCell<Option<pac::TIM3>>> = Mutex::new(RefCell::new(None));
static G_TIM2: Mutex<RefCell<Option<pac::TIM2>>> = Mutex::new(RefCell::new(None));
static STEPS_LEFT_3: AtomicU32 = AtomicU32::new(0);
static STEPS_LEFT_2: AtomicU32 = AtomicU32::new(0);

// Bufor na 11 bajtów: [0xFF, 0xFE, A0, A0, A0, A0, A1, A1, A1, A1, 0xFD]
static mut RX_BUFFER: [u8; 11] = [0u8; 11];

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Stepper System Booting...");

    let dp = pac::Peripherals::take().unwrap();
    
    // --- 1. RCC & GPIOA (PAC) ---
    let rcc_ptr = pac::RCC::ptr();
    let gpioa_ptr = pac::GPIOA::ptr();
    let gpiob_ptr = pac::GPIOB::ptr();

    unsafe {
        // Włączamy zegar dla GPIOA ORAZ GPIOB (bit IOPBEN)
        (*rcc_ptr).iopenr().modify(|_, w| w.iopaen().set_bit().iopben().set_bit());
        
        (*rcc_ptr).ahbenr().modify(|_, w| w.dmaen().set_bit());

        // ... twoja konfiguracja GPIOA ...

        // --- KONFIGURACJA PB0 i PB1 ---
        // MODER: 01 = Output mode
        (*gpiob_ptr).moder().modify(|_, w| {
            w.moder0().bits(0b01) // PB0 jako wyjście
             .moder1().bits(0b01) // PB1 jako wyjście
        });

        // OTYPER: 0 = Push-Pull (to jest domyślne po resecie, ale warto ustawić jawnie)
        (*gpiob_ptr).otyper().modify(|_, w| {
            w.ot0().push_pull() // PB0 Push-Pull
             .ot1().push_pull() // PB1 Push-Pull
        });
    }

    unsafe {
        (*rcc_ptr).iopenr().modify(|_, w| w.iopaen().set_bit());
        (*rcc_ptr).ahbenr().modify(|_, w| w.dmaen().set_bit());

        // Konfiguracja AF (PA0, PA7, PA9, PA10)
        (*gpioa_ptr).moder().modify(|_, w| {
            w.moder0().bits(0b10).moder7().bits(0b10).moder9().bits(0b10).moder10().bits(0b10)
        });
        (*gpioa_ptr).afrl().modify(|_, w| w.afrel0().bits(2).afrel7().bits(1));
        (*gpioa_ptr).afrh().modify(|_, w| w.afrel9().bits(1).afrel10().bits(1));
    }

    // --- 2. KONFIGURACJA HAL ---
    let mut rcc = dp.RCC.constrain();
    let gpioa_hal = dp.GPIOA.split(&mut rcc);
    
    // UART1
    let _uart = dp.USART1.usart(
        (gpioa_hal.pa9, gpioa_hal.pa10),
        FullConfig::default().baudrate(115200.bps()),
        &mut rcc,
    ).unwrap();

    // !!! KLUCZOWE: Włączamy komunikację UART -> DMA oraz przerwanie IDLE !!!
    unsafe {
        let usart1 = &*pac::USART1::ptr();
        usart1.cr3().modify(|_, w| w.dmar().set_bit());   // To pozwala UARTowi prosić DMA o transfer
        //usart1.cr1().modify(|_, w| w.idleie().set_bit()); // To włącza detekcję końca ramki
        usart1.rtor().write(|w| w.rto().bits(1152));
        usart1.cr2().modify(|_, w| w.rtoen().set_bit());
        usart1.cr1().modify(|_, w| w.rtoie().set_bit());

        pac::NVIC::unmask(pac::Interrupt::USART1);
    }

    let _clocks = rcc.freeze(hal::rcc::Config::default());
    
    unsafe {
        let rcc_ptr = pac::RCC::ptr();
        // TIM2 i TIM3 znajdują się na szynie APB (rejestr APBENR1)
        (*rcc_ptr).apbenr1().modify(|_, w| {
            w.tim2en().set_bit() // Włącz zegar TIM2
             .tim3en().set_bit() // Włącz zegar TIM3
        });
    }
    // --- 3. DMA & DMAMUX (PAC) ---
    unsafe {
        dp.DMAMUX.ccr(0).write(|w| w.dmareq_id().bits(50)); // 50 = USART1_RX

        let ch1 = dp.DMA1.ch1();
        ch1.par().write(|w| w.pa().bits(pac::USART1::ptr() as u32 + 0x24)); // Adres rejestru RDR
        ch1.mar().write(|w| w.ma().bits(addr_of_mut!(RX_BUFFER) as u32));
        ch1.ndtr().write(|w| w.ndt().bits(11));
        ch1.cr().write(|w| w.minc().set_bit().en().set_bit());
    }

let tim3 = dp.TIM3;
    // HSI = 16MHz. PSC = 15 => Zegar Timera = 1 MHz (1 us)
    tim3.psc().write(|w| unsafe { w.bits(15) }); 
    tim3.arr().write(|w| unsafe { w.arr().bits(8000) }); // Wartość startowa bezpieczna
    tim3.ccr2().write(|w| unsafe { w.ccr().bits(200) }); // Wartość startowa
    tim3.ccmr1_output().modify(|_, w| unsafe { w.oc2m().bits(0b110).oc2pe().set_bit() });
    tim3.ccer().modify(|_, w| w.cc2e().set_bit());
    tim3.dier().write(|w| w.uie().set_bit());
    // WAŻNE: Wymuś update event, aby załadować PSC do rejestru cienia
    tim3.egr().write(|w| w.ug().set_bit()); 

    let tim2 = dp.TIM2;
    tim2.psc().write(|w| unsafe { w.bits(15) }); // 1 MHz
    tim2.arr().write(|w| unsafe { w.bits(8000) });
    tim2.ccr1().write(|w| unsafe { w.bits(200) });
    tim2.ccmr1_output().modify(|_, w| unsafe { w.oc1m().bits(0b110).oc1pe().set_bit() });
    tim2.ccer().modify(|_, w| w.cc1e().set_bit());
    tim2.dier().write(|w| w.uie().set_bit());
    tim2.egr().write(|w| w.ug().set_bit());

    cortex_m::interrupt::free(|cs| {
        G_TIM3.borrow(cs).replace(Some(tim3));
        G_TIM2.borrow(cs).replace(Some(tim2));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM3);
        pac::NVIC::unmask(pac::Interrupt::TIM2);
    }

    info!("System up and running. Awaiting UART...");

    static LAST_POS0: AtomicU32 = AtomicU32::new(STEPS/2);
    static LAST_POS1: AtomicU32 = AtomicU32::new(STEPS/2);

    loop {
        // Sprawdzenie czy DMA skończyło (odebrano 11 bajtów)
        if dp.DMA1.isr().read().tcif1().bit_is_set() {
            dp.DMA1.ifcr().write(|w| w.ctcif1().set_bit()); 

            let buffer = unsafe { *addr_of_mut!(RX_BUFFER) };
            //info!("UART RX: {:x}", buffer);

            if buffer[0] == 0xff && buffer[1] == 0xfe && buffer[10] == 0xfd {
                let a0 = u32::from_be_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]);
                let a1 = u32::from_be_bytes([buffer[6], buffer[7], buffer[8], buffer[9]]);

                //tutaj dodać convertery do przeliczenia pozycji na kroki
                //dodać też odpowiednie piny do wstecznego i ewentualnie handlowanie
                //błędów z sterownika silnika krokowego
                
                let (steps0, rev0) = converter(&LAST_POS0,a0);
                let (steps1, rev1) = converter(&LAST_POS1,a1);
                

                unsafe {
                    let gpiob = pac::GPIOB::ptr();

                    // Obsługa kierunku dla Silnika 2 (PB0)
                    if rev0 {
                        (*gpiob).bsrr().write(|w| w.bs0().set_bit()); // PB0 HIGH
                    } else {
                        (*gpiob).bsrr().write(|w| w.br0().set_bit()); // PB0 LOW
                    }

                    // Obsługa kierunku dla Silnika 3 (PB1)
                    if rev1 {
                        (*gpiob).bsrr().write(|w| w.bs1().set_bit()); // PB1 HIGH
                    } else {
                        (*gpiob).bsrr().write(|w| w.br1().set_bit()); // PB1 LOW
                    }
                }

                //info!("Command: Axis0={}, Axis1={}", steps0, steps1);
                start_motor_3(steps0);
                start_motor_2(steps1);
            } else {
                warn!("Invalid frame received!");
            }

            restart_dma();
        }

        cortex_m::asm::wfi();
    }
}

// --- FUNKCJE POMOCNICZE ---

fn restart_dma() {
    unsafe {
        let dp = pac::Peripherals::steal();
        let ch1 = dp.DMA1.ch1();
        ch1.cr().modify(|_, w| w.en().clear_bit());
        ch1.ndtr().write(|w| w.ndt().bits(11));
        ch1.cr().modify(|_, w| w.en().set_bit());
    }
}

#[interrupt]
fn USART1() {
    unsafe {
        let dp = pac::Peripherals::steal();
        let isr = dp.USART1.isr().read();

        // Sprawdzamy flagę RTOF (Receiver Timeout) zamiast IDLE
        if isr.rtof().bit_is_set() {
            // Czyścimy flagę RTO (bit RTOCF w rejestrze ICR)
            dp.USART1.icr().write(|w| w.rtocf().set_bit());
            
            // Logika taka sama: sprawdzamy ile zostało w DMA
            let left = dp.DMA1.ch1().ndtr().read().ndt().bits();
            
            // Jeśli RTO wystąpiło, a bufor nie jest pełny (czyli DMA nie zdążyło pobrać 11 bajtów)
            // to znaczy, że transmisja się urwała w połowie.
            if left > 0 && left < 11 {
                warn!("RTO (Timeout) detected, incomplete frame (left: {}). Resetting DMA...", left);
                restart_dma();
            }
        }
    }
}

// --- POPRAWIONE FUNKCJE STARTUJĄCE SILNIKI ---

// Stała: Liczba cykli zegara 1MHz w czasie 16ms
const TICKS_PER_FRAME: u32 = 19_000; 
// Minimalne ARR - zabezpieczenie przed zawieszeniem CPU (np. ARR=1 dałoby 500kHz przerwań)
// ARR=40 daje max 25kHz steps/sec. Zmniejsz jeśli masz szybkie silniki i pewny zegar.
const MIN_ARR: u32 = 80; 

fn start_motor_3(steps: u32) {
    cortex_m::interrupt::free(|cs| {
        if let Some(tim3) = G_TIM3.borrow(cs).borrow().as_ref() {
            // 1. Zawsze najpierw zatrzymujemy timer
            tim3.cr1().modify(|_, w| w.cen().clear_bit());
            
            // 2. Jeśli brak kroków, kończymy (timer zostaje wyłączony)
            if steps == 0 {
                return;
            }

            // 3. Obliczamy ARR, aby ruch trwał ok. 16ms
            // Wzór: ARR = (TotalTime / Steps) - 1
            let mut arr = TICKS_PER_FRAME / steps;
            
            // Zabezpieczenie przed zbyt dużą prędkością (zbyt małym ARR)
            if arr < MIN_ARR {
                arr = MIN_ARR; 
                // Opcjonalnie: warn!("Speed Limit hit on M3!");
            }
            // Zabezpieczenie dla TIM3 (16-bit)
            if arr > 0xFFFF {
                arr = 0xFFFF;
            }

            // 4. Ustawiamy rejestry (ARR ustala częstotliwość, CCR wypełnienie ~50%)
            // Odejmujemy 1, bo licznik liczy od 0 do ARR
            let arr_val = (arr - 1) as u16;
            
            tim3.arr().write(|w| unsafe { w.arr().bits(arr_val) });
            tim3.ccr2().write(|w| unsafe { w.ccr().bits(arr_val / 2) }); // 50% duty cycle

            // 5. Reset licznika i flag
            tim3.cnt().write(|w| unsafe { w.bits(0) });
            STEPS_LEFT_3.store(steps, Ordering::SeqCst);
            tim3.sr().write(|w| w.uif().clear_bit());

            // 6. Start
            tim3.cr1().modify(|_, w| w.cen().set_bit());
        }
    });
}

fn start_motor_2(steps: u32) {
    cortex_m::interrupt::free(|cs| {
        if let Some(tim2) = G_TIM2.borrow(cs).borrow().as_ref() {
            tim2.cr1().modify(|_, w| w.cen().clear_bit());
            
            if steps == 0 { return; }

            let mut arr = TICKS_PER_FRAME / steps;
            if arr < MIN_ARR { arr = MIN_ARR; }
            // TIM2 jest 32-bitowy, więc nie musimy martwić się o górny limit tak bardzo jak w TIM3,
            // ale dla zachowania spójności czasowej max to i tak ~16ms.
            
            let arr_val = arr - 1;

            tim2.arr().write(|w| unsafe { w.bits(arr_val) });
            tim2.ccr1().write(|w| unsafe { w.bits(arr_val / 2) });

            tim2.cnt().write(|w| unsafe { w.bits(0) });
            STEPS_LEFT_2.store(steps, Ordering::SeqCst);
            tim2.sr().write(|w| w.uif().clear_bit());
            
            tim2.cr1().modify(|_, w| w.cen().set_bit());
        }
    });
}


// --- POPRAWIONE OBSŁUGI PRZERWAŃ ---

// #[interrupt]
// fn TIM3() {
//     cortex_m::interrupt::free(|cs| {
//         if let Some(tim3) = G_TIM3.borrow(cs).borrow().as_ref() {
//             // Czyścimy flagę poprzez write (bezpieczniej dla rc_w0)
//             tim3.sr().write(|w| w.uif().clear_bit());
            
//             let steps = STEPS_LEFT_3.load(Ordering::SeqCst);
//             if steps > 0 {
//                 STEPS_LEFT_3.store(steps - 1, Ordering::SeqCst);
//             } else {
//                 // Jeśli kroki się skończyły, wyłączamy timer
//                 tim3.cr1().modify(|_, w| w.cen().clear_bit());
//             }
//         }
//     });
// }

#[interrupt]
fn TIM3() {
    cortex_m::interrupt::free(|cs| {
        if let Some(tim3) = G_TIM3.borrow(cs).borrow().as_ref() {
            // Write 0 to clear (najbezpieczniejsza metoda)
            tim3.sr().write(|w| w.uif().clear_bit());
            
            let steps = STEPS_LEFT_3.load(Ordering::SeqCst);
            if steps > 1 {
                // Jeśli zostało więcej niż 1 krok, dekrementujemy
                STEPS_LEFT_3.store(steps - 1, Ordering::SeqCst);
            } else {
                // Ostatni krok wykonany (lub było 0) -> Stop
                tim3.cr1().modify(|_, w| w.cen().clear_bit());
                // Ustawiamy na 0 dla porządku
                STEPS_LEFT_3.store(0, Ordering::SeqCst);
            }
        }
    });
}
// Analogicznie dla TIM2...


#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(tim2) = G_TIM2.borrow(cs).borrow().as_ref() {
            // Write 0 to clear (najbezpieczniejsza metoda)
            tim2.sr().write(|w| w.uif().clear_bit());
            
            let steps = STEPS_LEFT_2.load(Ordering::SeqCst);
            if steps > 1 {
                // Jeśli zostało więcej niż 1 krok, dekrementujemy
                STEPS_LEFT_2.store(steps - 1, Ordering::SeqCst);
            } else {
                // Ostatni krok wykonany (lub było 0) -> Stop
                tim2.cr1().modify(|_, w| w.cen().clear_bit());
                // Ustawiamy na 0 dla porządku
                STEPS_LEFT_2.store(0, Ordering::SeqCst);
            }
        }
    });
}
// Analogicznie dla TIM2...