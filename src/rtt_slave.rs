use std::sync::mpsc;

use super::common::{MasterReq, SlaveRep};

pub fn run(rx: mpsc::Receiver<MasterReq>, tx: mpsc::Sender<SlaveRep>) {
    loop {
        match rx.recv() {
            Ok(MasterReq::Terminate) =>
                break,
            Err(mpsc::RecvError) =>
                break,
        }
    }
}