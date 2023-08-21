use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info,
};
#[tokio::main]
async fn main() -> Result<(), DynError>{
    let ctx = Context::new()?;
    let node = ctx.create_node("pole_detector", None, Default::default())?;
    //let publisher = node.create_publisher::<drobo_interfaces::msg::SolenoidStateMsg>("solenoid_order", None)?;

    //let logger = Rc::new(Logger::new("pole detector"));

    // Publisherの実装は距離センサの現物が届いてから
    Ok(())
}