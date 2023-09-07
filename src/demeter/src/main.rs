use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::std_msgs, pr_info,
};

fn main() -> Result<(), DynError>{
    let demeter_address = 0x06;

    let ctx = Context::new()?;
    let node = ctx.create_node("demeter", None, Default::default())?;

    let subscriber = node.create_subscriber::<std_msgs::msg::Bool>("demeter_state", None)?;

    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = demeter_address;
    msg.semi_id = 0;
    msg.mode = 2; //PWM
    msg.power = 300;

    let logger = Logger::new("demeter");

    let mut selector = ctx.create_selector()?;

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg|{
            pr_info!(logger, "収穫機構: {}", if _msg.data {"上昇"} else {"下降"});
            msg.phase = _msg.data; //TODO: やってみて逆だったら調整
            publisher.send(&msg).unwrap();
        }),
    );

    loop {
        selector.wait()?;
    }
}