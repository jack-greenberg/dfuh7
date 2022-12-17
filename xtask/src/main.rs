use std::{
    env, fs,
    path::{Path, PathBuf},
    process::{Command},
};

use std::io::Write;
use clap::Parser;
// use openssh::{Session, KnownHosts, Stdio};
// use openssh_sftp_client::Sftp;
// use ssh2::Session;
// use std::net::TcpStream;
use rsyn::Client;

use std::io::BufReader;
use std::io::prelude::*;

mod dist;

#[derive(Parser)]
enum Xtask {
    Dist {
        
    },
    Flash {

    },
    Debug {

    },
    RemoteFlash {
        remote_host: String
    },
}

fn main() {
    let task = env::args().nth(1);
    
    let xtask = Xtask::parse();

    match xtask {
        Xtask::RemoteFlash {
            remote_host,
        } => {
            build().unwrap();
            remote_copy(remote_host);
            // remote_flash();
        },
        _ => {
            panic!("Not implemented yet");
        }
    }
}

fn build() -> Result<(), Box<dyn std::error::Error>> {
    let cargo = env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());

    let status = Command::new(cargo)
        .args(&["build", "--release", "--package=dfuh7", "--target=thumbv7em-none-eabihf"])
        .status()?;

    
    if !status.success() {
        Err("cargo build failed")?;
    }

    Ok(())
}

fn remote_copy(remote_host: String) {
    let mut tcp = TcpStream::connect(remote_host.as_str()).unwrap();
    let mut session = Session::new().unwrap();
    session.set_tcp_stream(tcp);
    session.handshake().unwrap();

    session.userauth_agent("jack").unwrap();

    assert!(session.authenticated());

    let mut binary = fs::File::open("target/thumbv7em-none-eabihf/release/dfuh7").expect("No file found");

    let file_size = binary.metadata().unwrap().len();
    let mut file_buffer = vec![0; file_size as usize];
    binary.read(&mut file_buffer).expect("Buffer overflow");

    let mut remote_file = session.scp_send(Path::new("/tmp/dfuh7"), 0o644, file_size, None).unwrap();

    remote_file.write(&file_buffer).unwrap();

    remote_file.send_eof().unwrap();
    remote_file.wait_eof().unwrap();
    remote_file.close().unwrap();
    remote_file.wait_close().unwrap();
}
