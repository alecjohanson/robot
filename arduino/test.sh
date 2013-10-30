rosbag record -q -o stepresp --duration=25 /motion/PWM /motion/Encoders &
sleep 1
for p in $(seq 15 15 255); do echo -e "PWM1: -$p\nPWM2: $p\n---"; sleep 2; done | rostopic pub /motion/PWM differential_drive/PWM
sleep 2
for p in $(seq 15 15 255); do echo -e "PWM1: $p\nPWM2: -$p\n---"; sleep 2; done | rostopic pub /motion/PWM differential_drive/PWM
