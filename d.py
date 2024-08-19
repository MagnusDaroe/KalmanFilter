



dt = 0.1
samples = 10
gps_rate = 10

for i in range(samples+1):
   
   cond2 = int(dt/gps_rate)
   cond = int((1/gps_rate)/ dt)
   check = i % cond
   
   print(cond2, check, check == 0)
   