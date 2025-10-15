use std::io::Result;
fn main() -> Result<()> {
    prost_build::compile_protos(
        &[
            "src/proto/Blackboard.proto", /*, "src/naoData.proto", "src/Commands.proto"*/
        ],
        &["src/"],
    )?;
    Ok(())
}
