data = importdata("debug.txt");

dist = data(:, 1);
speed = data(:, 2);
steer = data(:, 3);

N = 30;

accel = data(:, 4);
accel = filter(ones(1, N)/N, 1, accel);

speed_diff = [0; data(2:end, 2) - data(1:end-1, 2)];
speed_diff = filter(ones(1, N)/N, 1, speed_diff);

ci = accel < -0.2;
mu = zeros(size(dist));
mu(ci) = speed_diff(ci) ./ accel(ci) .* speed(ci);

track_pos = zeros(size(dist));
track_pos(steer > 0.05) = 0.8;
track_pos(steer < -0.05) = -0.8;
track_pos = filter(ones(1, 50)/50, 1, [track_pos; zeros(50, 1)]);
track_pos = track_pos(26:end-25);

track_pos_diff = [0; track_pos(2:end) - track_pos(1:end-1)];

speed_new = speed;
speed_new(accel > 0.2) = 280;

last_speed = 0;
for i = flip(1:(length(speed_new)-1))
  if track_pos_diff(i) * track_pos(i) < 0
      %last_speed = (1 - abs(steer(i))) * 80;
      last_speed = 50;
      speed_new(i) = speed_new(i) + last_speed;
  end
  %speed_new(i) = speed_new(i) + last_speed;
end

plot(dist, speed);
hold on;
plot(dist, steer * 200);
plot(dist, accel * 200);
plot(dist, track_pos * 200);
plot(dist, track_pos_diff * 2000);
plot(dist, speed_new);
%plot(dist, mu);
%plot(data(1:end-1, 1), data_diff * 200);

out_data = [dist, speed_new, steer, accel, track_pos];
writematrix(out_data, "debug_in.txt", "Delimiter", " ");