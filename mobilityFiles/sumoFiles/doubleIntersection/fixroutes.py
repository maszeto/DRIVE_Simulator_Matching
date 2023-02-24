f1 = open("badtrips.xml", "r")
f2 = open("goodtrips.xml", "w")
linNum = 1
for line in f1.readlines():
    good = ' '.join(line.split(' ')[2:])
    gooder = '<trip id="{}" '.format(linNum)
    goody = gooder + good

    if("veh_passenger" in goody):
        goody = goody.replace("veh_passenger", "pas")
    else:
        goody = goody.replace("bus_bus", "amb")
    
    f2.write(goody)
    linNum += 1

f1.close()
f2.close()