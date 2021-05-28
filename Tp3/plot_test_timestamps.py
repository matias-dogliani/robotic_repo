N=10
plt.figure(figsize=(15, 3))
plt.title("Correspondencia entre timestamps")

plt.xlim(Scandata[0][0], Scandata[0][N])
plt.ylim(0,2.5)

for i in range(N): 
    plt.plot(Scandata[0][i],1, marker = '*', color = "blue")
 


for i in range(near_i[N]):  #maximo indice de Odometria que coincide con los n puntos de laser
     if i in near_i:       #Comparacion para los n primeros 
        plt.plot(Odomdata[0][i],1.2, marker = '.', color = "red")
    elif (i%2):
        plt.plot(Odomdata[0][i],1.2, marker = '.', color = "green")
    else: 
        pass

