function [] = PLOT(POSITION,VELOCITY,ACCELRATE)
figure(1)
hold on;
plot(POSITION(1,:));plot(POSITION(2,:));plot(POSITION(3,:));plot(POSITION(4,:));plot(POSITION(5,:));plot(POSITION(6,:));

figure(2)
hold on;
plot(VELOCITY(1,:));plot(VELOCITY(2,:));plot(VELOCITY(3,:));plot(VELOCITY(4,:));plot(VELOCITY(5,:));plot(VELOCITY(6,:));

figure(3)
hold on;
plot(ACCELRATE(1,:));plot(ACCELRATE(2,:));plot(ACCELRATE(3,:));plot(ACCELRATE(4,:));plot(ACCELRATE(5,:));plot(ACCELRATE(6,:));
end

