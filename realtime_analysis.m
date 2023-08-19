clc, clear, close all
lw = 2;

t4 = load("time_test4.mat");
t5 = load("time_test5.mat");
t10 = load("time_test10.mat");
t11 = load("time_test11.mat");
t12 = load("time_test12.mat");
t13 = load("time_test13.mat");
t14 = load("time_test14.mat");
t15 = load("time_test15.mat");
t16 = load("time_test16.mat");

t12sp = load("time_test12sp.mat");





t4 = t4.timer;
t5 = t5.timer;
t10 = t10.timer;
t11 = t11.timer;
t12 = t12.timer;
t13 = t13.timer;
t14 = t14.timer;
t15 = t15.timer;
t16 = t16.timer;
t12sp = t12sp.timer;

figure
plot(t12,'o-','LineWidth', lw), hold on
plot(t12sp,'o-','LineWidth', lw), hold on
plot(t13,'o-','LineWidth', lw)
plot(t14,'o-','LineWidth', lw)
grid on
legend("No objects (scenario no.0)", "1 Object (scenario no.1)", "3 Objects (scenario no.2)",...
    "12 Objects (scenario no.3)")
xlabel("Elapsed simulation time (s)")
ylabel("Computed time (s)")
title("Various scenarios with dynamic environmental constraints")
set(gca, 'FontSize', 24, 'LineWidth', 2)


figure
plot(t4,'o-','LineWidth', lw), hold on
plot(t10,'o-','LineWidth', lw)
plot(t15,'o-','LineWidth', lw)
grid on
legend("No environmental constraints", "Static environmental constraints",...
    "Dynamic environmental constraints")
xlabel("Elapsed simulation time (s)")
ylabel("Computed time (s)")
title("Scenario no.4 : 1 static obstacle + 2 dynamic obstacles")
set(gca, 'FontSize', 24, 'LineWidth', 2)


figure
plot(t5,'o-','LineWidth', lw), hold on
plot(t11,'o-','LineWidth', lw)
plot(t16,'o-','LineWidth', lw)
grid on
legend("No environmental constraints", "Static environmental constraints",...
    "Dynamic environmental constraints")
xlabel("Elapsed simulation time (s)")
ylabel("Computed time (s)")
title("Scenario no.5 : 2 static obstacles + 2 dynamic obstacles")
set(gca, 'FontSize', 24, 'LineWidth', 2)

