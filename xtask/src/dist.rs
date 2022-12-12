use std::{
    env, fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

pub fn build() -> Result<(), Box<dyn std::error::Error>> {
    let cargo = env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());

    let status = Command::new(cargo)
        .current_dir(project_root())
        .args(&["build", "--release", "--package=dfuh7", "--target=thumbv7em-none-eabihf"])
        .status()?;

    
    if !status.success() {
        Err("cargo build failed")?;
    }

    Ok(())
}

fn project_root() -> PathBuf {
    Path::new(&env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(1)
        .unwrap()
        .to_path_buf()
}
