m=1 %mass of vehicle
b=10 
k=20
num=1
den=[m,b,k]
plant=tf(num,den)
step(plant)
ki=300
kp=100
contr=tf([kp,ki],[1,0])
sys_ctrl=feedback(contr*plant,1)
t=0:0.01:4
step(sys_ctrl,t)