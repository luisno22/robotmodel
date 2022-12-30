import numpy as np
import matplotlib.pyplot as plt
import sys

file = "plot"
if len(sys.argv)>1:
	print(sys.argv[0])
	file = sys.argv[1]
	axis = file[0]
	ficherox = open('vector_x_'+ file +'.txt')
	ficheroy = open('vector_y_'+ file +'.txt')
else:
	print("hola")
	axis = str(3)
	ficherox = open('vectorx.txt')
	ficheroy = open('vectory.txt')

print("Ejes: " + axis)

x = [0]
y = [0]
while x[-1] != '':
	x.append(ficherox.readline()[0:-1])
	y.append(ficheroy.readline()[0:-1])


x = x[0:-1]
y = y[0:-1]


x_float = [float(z) for z in x]
y_float = [float(z) for z in y]


plt.figure(1)
plt.plot(x_float,y_float)
#plt.show()

#print(x)
#print(y)


ficherox.close()
ficheroy.close()

sub_x = [0]
sub_y = [0]

plt.figure(2)
k=0
for i in range(0,len(x_float)):
	if x_float[i]==0 and y_float[i]==0:
		plt.plot(sub_x,sub_y,linewidth=1.5)
		k = k + 1
		sub_y.clear()
		sub_x.clear()
		print(i)
	else:
		sub_x.append(x_float[i])
		sub_y.append(y_float[i])



#plt.plot(x_float,y_float,'o', markersize=1)
plt.title(file)
plt.axis([-int(axis), int(axis), -int(axis), int(axis)])
plt.legend()
#plt.show()


coeffs_matrix = []
# Derecha 1
coeffs_matrix.append([0,1.43616,-0.651065])
coeffs_matrix.append([0,-2.8708,1.38785])

# Derecha 2
coeffs_matrix.append([0,2.5477,-0.997032])
coeffs_matrix.append([0,-17.2879,29.6945])

# centro
coeffs_matrix.append([0,-63.7281,-106.188])
coeffs_matrix.append([0,-74.3563,1090.69])

# izquierda
coeffs_matrix.append([0,-2.5477,-0.997032])
coeffs_matrix.append([0,6.60754,6.4482])
coeffs_matrix.append([0,-1.40824,-0.563836])
coeffs_matrix.append([0,1.95973, 0.944646])
#y = np.array([np.sum(np.array([coeffs_matrix[4][i]*(j**i) for i in range(len(coeffs_matrix[4]))])) for j in x])

plt.figure(3)
x = np.linspace(0, 2, 60)

for k in range(0,len(coeffs_matrix)):
	d = [0]
	print(k)
	if k>4:
		x = np.linspace(-2, 0, 60)
	else:
		x = np.linspace(0, 2, 60)

	y = np.array([np.sum(np.array([coeffs_matrix[k][i]*(j**i) for i in range(len(coeffs_matrix[k]))])) for j in x])
	if k>4:
		x = x[::-1]
		y = y[::-1]
		print(x)
		print(y)
	print(x)
	for i in range(1,len(x)):
		print("distancia " + str(np.hypot(x[i],y[i])))
		if np.hypot(x[i],y[i])>1.9:
			print("superior a 2: "+str(i))
			x = x[0:i+1]
			y = y[0:i+1]
			print(x)
			print(y)
			break
	if k==2:
		print(y)
	plt.plot(x,y)

print(coeffs_matrix[0])
plt.axis([-3,3,-3,3])
#plt.plot(x, y)
plt.show()
