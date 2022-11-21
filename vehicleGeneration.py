import random

def generate_vehicles():

    template_str = '<vehicle id="{}" type="{}" route="route0" depart="{}" departLane = "{}" color="{}"/>'


    for i in range(300):

        if(i % 4 == 0):
            vehicleType = "amb"
            color = "0,1,0"
        else:
            vehicleType = "pas"
            color = "1,0,0"
        vehicleID = i 

        departLane = random.randint(1,4)
        depart = int(1 / 10)

        print(template_str.format(vehicleID, vehicleType, depart, departLane, color))


if __name__ == "__main__":
    generate_vehicles()