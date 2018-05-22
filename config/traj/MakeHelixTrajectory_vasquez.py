import math;

def fmt(value):
    return "%.3f" % value

period = 5
radius = 1.5
timestep = 0.02
maxtime = period*2
z = -1

with open('HelixVelYaw.txt', 'w') as the_file:
    t=0;
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period) * radius;
        y = math.cos(t * 2 * math.pi / period) * radius; 

        t_mas_1 = t+timestep;
        x_next = math.sin(t_mas_1 * 2 * math.pi / period) * radius;
        y_next = math.cos(t_mas_1 * 2 * math.pi / period) * radius;
        z_next = z - 0.005
        vx = (x_next - x)/timestep;
        vy = (y_next - y)/timestep;
        vz = (z_next - z)/timestep;

        roll = 0.0;
        pitch = 0.0;
        yaw = -t * 2 * math.pi / period;

        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z));
        the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + fmt(vz));
        the_file.write("," + fmt(yaw) + "," + fmt(pitch) + "," + fmt(roll) + "\n");
        t += timestep;
        z -= 0.005 
