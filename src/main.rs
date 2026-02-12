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


// --- ZMIENNE GLOBALNE ---
static G_TIM3: Mutex<RefCell<Option<pac::TIM3>>> = Mutex::new(RefCell::new(None));
static G_TIM2: Mutex<RefCell<Option<pac::TIM2>>> = Mutex::new(RefCell::new(None));
static STEPS_LEFT_3: AtomicU32 = AtomicU32::new(0);
static STEPS_LEFT_2: AtomicU32 = AtomicU32::new(0);

// Bufor na 11 bajtów: [0xFF, 0xFE, A0, A0, A0, A0, A1, A1, A1, A1, 0xFD]
static mut RX_BUFFER: [u8; 11] = [0u8; 11];

//ZMIENNA DLA KONWERSJI U32 W KROKI
static LASTPOS: AtomicU32 = AtomicU32::new(STEPS/2); //zakładany że silnik jest zbazowany,

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Stepper System Booting...");

    let dp = pac::Peripherals::take().unwrap();
    
    // --- 1. RCC & GPIOA (PAC) ---
    let rcc_ptr = pac::RCC::ptr();
    let gpioa_ptr = pac::GPIOA::ptr();
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
        usart1.cr1().modify(|_, w| w.idleie().set_bit()); // To włącza detekcję końca ramki
        pac::NVIC::unmask(pac::Interrupt::USART1);
    }

    let _clocks = rcc.freeze(hal::rcc::Config::default());

    // --- 3. DMA & DMAMUX (PAC) ---
    unsafe {
        dp.DMAMUX.ccr(0).write(|w| w.dmareq_id().bits(50)); // 50 = USART1_RX

        let ch1 = dp.DMA1.ch1();
        ch1.par().write(|w| w.pa().bits(pac::USART1::ptr() as u32 + 0x24)); // Adres rejestru RDR
        ch1.mar().write(|w| w.ma().bits(addr_of_mut!(RX_BUFFER) as u32));
        ch1.ndtr().write(|w| w.ndt().bits(11));
        ch1.cr().write(|w| w.minc().set_bit().en().set_bit());
    }

    // --- 4. TIMERY (PAC) ---
    let tim3 = dp.TIM3;
    tim3.psc().write(|w| unsafe { w.bits(15) });
    tim3.arr().write(|w| unsafe { w.arr().bits(1000) });
    tim3.ccr2().write(|w| unsafe { w.ccr().bits(500) });
    tim3.ccmr1_output().modify(|_, w| unsafe { w.oc2m().bits(0b110).oc2pe().set_bit() });
    tim3.ccer().modify(|_, w| w.cc2e().set_bit());
    tim3.dier().write(|w| w.uie().set_bit());

    let tim2 = dp.TIM2;
    tim2.psc().write(|w| unsafe { w.bits(15) });
    tim2.arr().write(|w| unsafe { w.bits(2000) });
    tim2.ccr1().write(|w| unsafe { w.bits(1000) });
    tim2.ccmr1_output().modify(|_, w| unsafe { w.oc1m().bits(0b110).oc1pe().set_bit() });
    tim2.ccer().modify(|_, w| w.cc1e().set_bit());
    tim2.dier().write(|w| w.uie().set_bit());

    cortex_m::interrupt::free(|cs| {
        G_TIM3.borrow(cs).replace(Some(tim3));
        G_TIM2.borrow(cs).replace(Some(tim2));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM3);
        pac::NVIC::unmask(pac::Interrupt::TIM2);
    }

    info!("System up and running. Awaiting UART...");

    loop {
        // Sprawdzenie czy DMA skończyło (odebrano 11 bajtów)
        if unsafe { dp.DMA1.isr().read().tcif1().bit_is_set() } {
            unsafe { dp.DMA1.ifcr().write(|w| w.ctcif1().set_bit()); }

            let buffer = unsafe { *addr_of_mut!(RX_BUFFER) };
            info!("UART RX: {:x}", buffer);

            if buffer[0] == 0xff && buffer[1] == 0xfe && buffer[10] == 0xfd {
                let a0 = u32::from_le_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]);
                let a1 = u32::from_le_bytes([buffer[6], buffer[7], buffer[8], buffer[9]]);

                //tutaj dodać convertery do przeliczenia pozycji na kroki
                //dodać też odpowiednie piny do wstecznego i ewentualnie handlowanie
                //błędów z sterownika silnika krokowego
                
                info!("Command: Axis0={}, Axis1={}", a0, a1);
                start_motor_3(a0);
                start_motor_2(a1);
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
        if dp.USART1.isr().read().idle().bit_is_set() {
            dp.USART1.icr().write(|w| w.idlecf().set_bit()); // Czyścimy flagę IDLE
            
            let left = dp.DMA1.ch1().ndtr().read().ndt().bits();
            if left > 0 && left < 11 {
                warn!("IDLE detected, frame incomplete. Resetting DMA...");
                restart_dma();
            }
        }
    }
}

fn start_motor_3(steps: u32) {
    if steps == 0 { return; }
    STEPS_LEFT_3.store(steps, Ordering::SeqCst);
    cortex_m::interrupt::free(|cs| {
        if let Some(tim3) = G_TIM3.borrow(cs).borrow().as_ref() {
            tim3.cnt().write(|w| unsafe { w.bits(0) });
            tim3.cr1().modify(|_, w| w.cen().set_bit());
        }
    });
}

fn start_motor_2(steps: u32) {
    if steps == 0 { return; }
    STEPS_LEFT_2.store(steps, Ordering::SeqCst);
    cortex_m::interrupt::free(|cs| {
        if let Some(tim2) = G_TIM2.borrow(cs).borrow().as_ref() {
            tim2.cnt().write(|w| unsafe { w.bits(0) });
            tim2.cr1().modify(|_, w| w.cen().set_bit());
        }
    });
}

#[interrupt]
fn TIM3() {
    cortex_m::interrupt::free(|cs| {
        if let Some(tim3) = G_TIM3.borrow(cs).borrow().as_ref() {
            tim3.sr().modify(|_, w| w.uif().clear_bit());
            let steps = STEPS_LEFT_3.load(Ordering::SeqCst);
            if steps > 0 {
                STEPS_LEFT_3.store(steps - 1, Ordering::SeqCst);
            } else {
                tim3.cr1().modify(|_, w| w.cen().clear_bit());
            }
        }
    });
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(tim2) = G_TIM2.borrow(cs).borrow().as_ref() {
            tim2.sr().modify(|_, w| w.uif().clear_bit());
            let steps = STEPS_LEFT_2.load(Ordering::SeqCst);
            if steps > 0 {
                STEPS_LEFT_2.store(steps - 1, Ordering::SeqCst);
            } else {
                tim2.cr1().modify(|_, w| w.cen().clear_bit());
            }
        }
    });
}