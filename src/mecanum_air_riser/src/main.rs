use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info, topic::subscriber::Subscriber
};
#[tokio::main]
async fn main() -> Result<(), DynError>{
    let ctx = Context::new()?;
    let node = ctx.create_node("mecanum_air_riser", None, Default::default())?;
    //let publisher = node.create_publisher::<drobo_interfaces::msg::SolenoidStateMsg>("solenoid_order", None)?;

    //let logger = Rc::new(Logger::new("mecanum air riser"));

    // Publisherの実装は距離センサの現物が届いてから

    let subscriber = node.create_subscriber::<drobo_interfaces::msg::SolenoidStateMsg>("solenoid_order", None)?;
    let _ = tokio::spawn(receiver(subscriber)).await;
    Ok(())
}

async fn receiver(mut subscriber: Subscriber<drobo_interfaces::msg::SolenoidStateMsg>) -> Result<(), DynError> {
    let logger = Logger::new(subscriber.get_topic_name());

    loop{
        let msg = subscriber.recv().await?;
        pr_info!(logger, "{}を{}", msg.axle_position, msg.state);
        // TODO: motor_libでソレノイドドライバを叩く
    }
}