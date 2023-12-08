#[derive(Debug, Clone, Default)]
enum MqttEvent {
    #[default]
    Connected,
    Disconnected,
    Publish(String, Vec<u8>),
}

#[derive(Debug, Clone, Default)]
enum MqttCmd {
    #[default]
    Connect,
    Disconnect,
    Publish(String, Vec<u8>),
    Subscribe(String),
    Unsubscribe(String),
}
struct Mqtt {
    pub actor: Actor<MqttCmd, MqttEvent>,
}

impl Mqtt {
    fn new() -> Self {
        Mqtt {
            actor: Actor::new(10),
        }
    }
    async fn run(&mut self) {
        loop {
            let mut cmd = self.actor.recv().await;
            match cmd {
                MqttCmd::Connect => {
                    self.actor.emit(&MqttEvent::Connected);
                    // connect
                    // emit event
                }
                MqttCmd::Disconnect => {
                    // disconnect
                    // emit event
                }
                MqttCmd::Publish(topic, payload) => {
                    // publish
                    // emit event
                }
                MqttCmd::Subscribe(topic) => {
                    // subscribe
                    // emit event
                }
                MqttCmd::Unsubscribe(topic) => {
                    // unsubscribe
                    // emit event
                }
            }
        }
    }
}

