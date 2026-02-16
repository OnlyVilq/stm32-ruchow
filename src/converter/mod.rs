use portable_atomic::{AtomicU32, Ordering};

pub const STEPS:u32 = 8000;

fn step_converter(position: u32) -> u32 {

    let pos = position as u64;
    let stp = STEPS as u64;
    let max = u32::MAX as u64;


    ((pos * stp + max / 2) / max) as u32
    //(pos as i64 + i32::MIN as i64) as i32
}

pub fn converter(atom_pos: &AtomicU32, position: u32) -> (u32,bool) {
    
    let pos_convert = step_converter(position) as i32;
    let last_pos = atom_pos.swap(pos_convert as u32,Ordering::Relaxed) as i32;
    let steps = (last_pos-pos_convert) as i32;
    let rev = steps > 0;

    let steps = steps.abs() as u32;
    (steps, rev)
}
