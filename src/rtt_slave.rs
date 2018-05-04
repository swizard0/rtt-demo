use std::sync::mpsc;

use super::common::{MasterPacket, SlavePacket};

pub fn run(rx: mpsc::Receiver<MasterPacket>, tx: mpsc::Sender<SlavePacket>) {
    run_idle(&rx, &tx);
}

fn run_idle(rx: &mpsc::Receiver<MasterPacket>, tx: &mpsc::Sender<SlavePacket>) {
    loop {
        match rx.recv() {
            Ok(MasterPacket::Terminate) =>
                break,
            Ok(MasterPacket::Interrupt) =>
                (),
            Err(mpsc::RecvError) =>
                break,
        }
    }
}
