f = open("output.txt", "r")

count1 = 0
count2=0
for line in f.readlines():
    if "amb" in line:
        count1 += 1
    if "pas" in line:
        count2 +=1

print(count1)
print(count2)
print(count1/count2)