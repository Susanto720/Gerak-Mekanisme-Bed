import streamlit as st
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
import math

st.title("Simulasi Gerakan Tempat Tidur Rumah Sakit")

st.write("### Input Data Gerak Awal Bed")
col1, col2 = st.columns(2)
pembagi = col1.number_input("Size", min_value=1, value=10)
beban_pasang = col1.number_input("Beban Bed (kg)", min_value=10, value=250)
alph_awal = (col2.number_input("Sudut Crank Awal(mm)", value=-30))/57.3
alph_akhir = (col2.number_input("Sudut Crank Akhir (mm)",value=45))/57.3

st.write("### Input Parameter Mekanisme Bed")

col1, col2= st.columns(2)
jarak_ground = col1.number_input("Tinggi Bed Terhadap Ground (mm)", min_value=200, value=232)
PCDawal = col1.number_input("PCD motor (mm)", min_value=390, value=410)
jarak_tumpuan = col1.number_input("Panjang Batang Utama (mm)",min_value=1000, value=1200)
eta = col1.number_input("Sudut Eta",min_value=25,value=39)
delta = col2.number_input("Sudut Delta", min_value=90, value=96)
AC = col2.number_input("Panjang Crank AC = BD (mm)",  value=280)
CE = col2.number_input("Panjang Crank CE = DF (mm)", value=124)
DH = col2.number_input("Posisi Motor di Crank DH (mm)",value=60)
DF = CE
BD = AC
alpha=np.linspace(alph_awal,alph_akhir,pembagi)
W=-10*beban_pasang*np.ones(pembagi)

Cx = []
Cy = []
Dx = []
Ex = [] 
Ey = []
Fx = []
Gx = []
Gy = []
Hx = [] 
Hy = []  
GHx= [] 
GHy = [] 
beta = []
Kx = []
Ky = []


## Menentukan Koordinat Joint
Ax=np.zeros(pembagi)
Ay=jarak_ground*np.ones(pembagi)
Bx=jarak_tumpuan*np.ones(pembagi)
By=jarak_ground*np.ones(pembagi)

for i in range(len(alpha)): 
    Cx.append(AC*math.cos(alpha[i])) 
    Cy.append(jarak_ground+AC*math.sin(alpha[i])) 
    Dx.append(jarak_tumpuan+Cx[i]) 
    i += 1
Cy1=np.array(Cy)
Cx1=np.array(Cx)
Dy=Cy1
AEE=AC**2+CE**2-2*AC*CE*math.cos(delta/57.3)
AE=AEE**0.5
gam1=(AC**2+AE**2-CE**2)/(2*AE*280)
gama1 = math.acos(gam1)

for i in range(len(alpha)): 
    Ex.append(AE*math.cos(gama1+alpha[i])) 
    Ey.append(jarak_ground+AE*math.sin(gama1+alpha[i])) 
    Fx.append(jarak_tumpuan+Ex[i])
    i += 1
Fy=Ey;
for i in range(len(alpha)): 
   Gx.append(Fx[i]-542)
   Gy.append(Fy[i]-95.7)

BHH=BD**2+DH**2-2*BD*DH*math.cos(eta/57.3)
BH=BHH**0.5
gam=(BD**2+BH**2-DH**2)/(2*BH*BD)
gama= math.acos(gam)

for i in range(len(alpha)): 
    Hx.append(jarak_tumpuan + BH*math.cos(alpha[i]-gama)) 
    Hy.append(jarak_ground+BH*math.sin(alpha[i]-gama)) 
    i += 1

Hx1=np.array(Hx)
Hy1=np.array(Hy)
Gx1=np.array(Gx)
Gy1=np.array(Gy)
Ty=Gy1;
Tx=np.array(Ex)+152;
GHx=Hx1-Gx1
GHy=Hy1-Gy1
GHH=GHx**2+GHy**2
GH=GHH**0.5

for i in range(len(alpha)): 
    beta.append(math.asin(GHy[i]/GH[i])) 
    i += 1

st.write("##### Menentukan Koordinat Platform Matras")

Dx1=np.array(Dx)
Lx=Cx1;
Ly=Cy1+200;
Dy1=Dy
Mx=Dx1
My=Dy1+200
Ox=Lx-400
Oy=Ly;
Qx=Mx+400
Qy=My


st.write("##### Menentukan Tinggi Angkat Tempat Tidur")

tinggi_angkat=My

st.write("##### Menentukan Langkah Spindel Aktuator")

stroke=GH-PCDawal


st.write("##### Menentukan Gaya Joint")

Ww=np.array(W)
FC=Ww/2
FD=FC
FD1=np.array(FD)
RBy=-Ww*(Cx1+0.5*jarak_tumpuan)/jarak_tumpuan;
RAy=-Ww-RBy;
REy=-FC-RAy;
RE=Cx*FC+Ex*REy
REx=RE/(Ey-Ay)
RAx=-REx
RBx=-RAx

st.write("##### Menentukan Gaya Dorong Motor")

Fx1=np.array(Fx)
Fy1=np.array(Fy)

for i in range(len(alpha)): 
     Kx.append(math.cos(beta[i]))
     Ky.append(math.sin(beta[i]))        
     i += 1  
    
Kx1=np.array(Kx)
Ky1=np.array(Ky)   
faktor_beta= -Kx1*(Hy1-Fy1)+Ky1*(Hx1-Fx1)

Bx1=np.array(Bx)
By1=np.array(By)

RH=-((Bx1-Fx1)*RBy-(By1-Fy1)*RBx+(Dx1-Fx1)*FD1);
R=RH/faktor_beta;
#R=-R;
Ry=R*Ky1
Rx=R*Kx1
RFy=-FD1-RBy-Ry
RF=(Dx1-jarak_tumpuan)*FD1+(Fx1-jarak_tumpuan)*RFy+(Hx1-jarak_tumpuan)*Ry-Hy1*Rx
RFx=RF/Fy1

st.write("##### Output Gerakan Pengaturan")

a1=57.3*alpha    
a2=GH            
a3=stroke        
a4=R            
a5=tinggi_angkat   
beta1=np.array(beta)  
a6=57.3*beta1

st.write("### Tampilan Gerakan Tempat Tidur ")
st.write("#### Plot Tinggi Angkat/ Panjang Aktuator VS Langkah Aktuator ")

fig, ax = plt.subplots()
# fig(figsize=(6, 4))
ax.plot(a3,a2,'r-d',label=r'Panjang Aktuator')
ax.plot(a3,a5,'b--o',label=r'Tinggi Angkat')
ax.set_xlabel('Langkah Aktuator (mm)')
ax.set_ylabel('Tinggi Angkat/ Panjang Aktuator(mm)')
ax.set_title('Langkah Aktuator Vs Tinggi Angkat')
ax.legend()

st.pyplot(fig)

st.write("#### Plot Gaya Motor terhadap Langkah Engkol")

fig, ax = plt.subplots()
# fig(figsize=(6, 4))
ax.bar(a1,R, width=4, edgecolor="white", linewidth=0.7)
ax.plot(a1,R,'k--d',label=r'Gaya Motor (N)')
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_title('Sine Wave')
ax.set_xlabel('Gerak Engkol (derajat)')
ax.set_ylabel('Gaya Motor (N)')
ax.set_title('Gerak Engkol  Vs Gaya Motor')
ax.legend()

st.pyplot(fig)

st.write("#### Tabel Data Output Gerakan Pengaturan")

data1 = [a1,a2,a3,a4,a5,a6]
data=np.transpose(data1)
data_df = pd.DataFrame(data)
data_df.columns = ["Putaran Engkol(Derajat)","Motor PCD (mm)","Motor Stroke (mm)","Motor Force (N)","Tinggi Angkat(mm)","Beta (derajat)"]
data_df

st.write("#### Tabel Gaya Reaksi Pada Elemen Mekanisme dititik Nodal")

data2 = [a1,57.3*gama1+a1,R, RAx, RAy,RBx,RBy,FD,REx,REy,RFx,RFy]
Data = np.transpose(data2)
Data_df = pd.DataFrame(Data,columns = ['Rotasi alfa', 'Rotasi Gama', 'MotorForce (N)', 'RAx (N)', 
                                       'RAy (N)', 'RBx (N)','RBy (N)','RC = RD (N)','REx (N)','REy (N)','RFx (N)','RFy (N)'])
Data_df

st.write("#### Plot Gaya Gaya Elemen Mekanisme Terhadap Langkah Engkol")
fig, ax = plt.subplots()

ax.plot(data2[1], R, "r-d", label=r"Motor Force")
ax.plot(data2[1],FD, "c-*", label=r"Reaksi Dukungan RC dan RD")
ax.plot(data2[1],RAy, "g-o",label=r"Reaksi Dukungan RAy")
ax.plot(data2[1], RAx ,"b--s", label=r"Reaksi Dukungan RAx= RBx=REx")
ax.plot(data2[1],REy, "r--p", label=r"Reaksi Dukungan REy")
ax.plot(data2[1],RBy,"k-8", label=r"Reaksi Dukungan RBy")
ax.plot(data2[1],RFy, "m-p", label=r"Reaksi Dukungan RFy")
ax.plot(data2[1],RFx, "y-h", label=r"Reaksi Dukungan RFx")

ax.set_ylabel("Gaya Motor Dan Gaya Reaksi (N)")               
ax.set_xlabel("Rotasi Crank Sisi Luar Gama (derajat)")
ax.set_title('Relasi Rotasi Crank dengan Gaya Motor Dan Reaksi Dukungan', fontsize=14)
ax.legend()
# Display the plot in Streamlit
st.pyplot(fig)

st.write("## Animasi Gerakan Tempat Tidur")

kkx1=[]
kky1=[]

for i in range(len(alpha)): 
     kkx1.append(PCDawal*math.cos(beta[i]))
     kky1.append(PCDawal*math.sin(beta[i]))        
     i += 1  
kkx=Gx1+np.array(kkx1)
kky=Gy1+np.array(kky1)
rds=75
d = rds*2
tinggi_ground = 232
A1x=Ax
A1y=Ay- tinggi_ground + rds
B1x=Bx
B1y=By- tinggi_ground + rds
px=A1x-rds
py=A1y-rds
px1=B1x-rds
py1=B1y-rds


st.write("#### Plot Posisi Tempat Tidur Pada Posisi Tertinggi ")

pembagi = pembagi - 1 
fig, ax = plt.subplots()

ax.axis("equal")
ax.plot([Ax[pembagi],Bx[pembagi]], [Ay[pembagi],By[pembagi]],'ko-',linewidth=3)
ax.plot([Ax[pembagi],Cx[pembagi]], [Ay[pembagi],Cy[pembagi]], 'bo-',linewidth=3)
ax.plot([Cx[pembagi],Ex[pembagi]], [Cy[pembagi],Ey[pembagi]], 'bo-',linewidth=3)
ax.plot([Ax[pembagi],Ex[pembagi]], [Ay[pembagi],Ey[pembagi]], 'b-',linewidth=2)
ax.plot([Bx[pembagi],Dx[pembagi]], [By[pembagi],Dy[pembagi]], 'bo-',linewidth=3) 
ax.plot([Dx[pembagi],Fx[pembagi]], [Dy[pembagi],Fy[pembagi]], 'bo-',linewidth=3)
ax.plot([Gx[pembagi],Fx[pembagi]], [Gy[pembagi],Fy[pembagi]], 'bo-',linewidth=3) 
ax.plot([Bx[pembagi],Fx[pembagi]], [By[pembagi],Fy[pembagi]], 'b-',linewidth=3)
ax.plot([Bx[pembagi],Hx1[pembagi]], [By[pembagi],Hy1[pembagi]], 'bo-',linewidth=2)
ax.plot([Ax[pembagi],A1x[pembagi]], [Ay[pembagi],A1y[pembagi]], 'ko-',linewidth=2) 
ax.plot([Bx[pembagi],B1x[pembagi]], [By[pembagi],B1y[pembagi]], 'ko-',linewidth=2) 
ax.add_patch(Circle((A1x[0],A1y[0]),radius=rds,fc='g'))
ax.add_patch(Circle((B1x[0],B1y[0]),radius=rds,fc='g'))
# c = Circle((A1x[pembagi],A1y[pembagi]),radius=rds)
# c1= Circle((B1x[pembagi],B1y[pembagi]),radius=rds)
# ax.gca().add_artist(c1)
ax.plot([Cx[pembagi],Lx[pembagi]], [Cy[pembagi],Ly[pembagi]], 'mo-',linewidth=3) 
ax.plot([Dx[pembagi],Mx[pembagi]], [Dy[pembagi],My[pembagi]], 'mo-',linewidth=3) 
ax.plot([Ox[pembagi],Lx[pembagi]], [Oy[pembagi],Ly[pembagi]], 'mo-',linewidth=3) 
ax.plot([Lx[pembagi],Mx[pembagi]], [Ly[pembagi],My[pembagi]], 'mo-',linewidth=3)
ax.plot([Mx[pembagi],Qx[pembagi]], [My[pembagi],Qy[pembagi]], 'mo-',linewidth=3)
ax.plot([Hx[pembagi],Gx1[pembagi]], [Hy[pembagi],Gy1[pembagi]], 'ko-',linewidth=3) 
ax.plot([Fx[pembagi],Gx1[pembagi]], [Fy[pembagi],Gy1[pembagi]], 'ro-',linewidth=3) 
ax.plot([Gx1[pembagi],Tx[pembagi]], [Gy1[pembagi],Ty[pembagi]], 'ro-',linewidth=3) 
ax.plot([Tx[pembagi],Ex[pembagi]], [Ty[pembagi],Ey[pembagi]], 'ro-',linewidth=3) 
ax.plot([Dx[pembagi],Hx[pembagi]], [Dy[pembagi],Hy[pembagi]], 'b-',linewidth=2) 
ax.plot([Hx[pembagi],kkx[pembagi]],[Hy[pembagi],kky[pembagi]], 'ro-',linewidth=3)
ax.set_title('Tempat Tidur Pada Posisi Tertinggi')
ax.set_xlabel('Span Panjang Bed (mm)')
ax.set_ylabel('Tinggi Angkat (mm)')
st.pyplot(fig)

st.write("##### Plot Posisi Tempat Tidur Pada Posisi Terendah ")

from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import matplotlib
import tempfile

fig, ax = plt.subplots()
ax.set_xlim(-400, 2000)
ax.set_ylim(-50, 800)
ax.plot([Ax[0],A1x[0]], [Ay[0],A1y[0]],color = 'k', lw=3)
ax.plot([A1x[0]],[A1y[0]],color = 'r', lw=5, marker='o')
ax.plot([Bx[0],B1x[0]], [By[0],B1y[0]],color = 'k', lw=3)
ax.plot([B1x[0]], [B1y[0]],color = 'r', lw=5, marker='o')
ax.plot([Ax[0],Bx[0]], [Ay[0],By[0]],color = 'k', lw=3, marker='o')
ax.add_patch(Circle((A1x[0],A1y[0]),radius=rds,fc='g'))
ax.add_patch(Circle((B1x[0],B1y[0]),radius=rds,fc='g'))
line1, =ax.plot([Ax[0],Cx[0]], [Ay[0],Cy[0]],color = 'b', lw=3 ,marker='o')    
line2, =ax.plot([Cx[0],Ex[0]], [Cy[0],Ey[0]],color = 'b', lw=3)
line3, =ax.plot([Ax[0],Ex[0]], [Ay[0],Ey[0]],color = 'b', lw=3,marker='o' )
line4, =ax.plot([Bx[0],Dx[0]], [By[0],Dy[0]],color = 'b', lw=3,marker='o' ) 
line5, =ax.plot([Dx[0],Fx[0]], [Dy[0],Fy[0]],color = 'b', lw=3)    
line6, =ax.plot([Gx[0],Fx[0]], [Gy[0],Fy[0]],color = 'b', lw=3)   
line7, =ax.plot([Bx[0],Fx[0]], [By[0],Fy[0]],color = 'b', lw=3,marker='o') 
line8, = ax.plot([Bx[0],Hx1[0]], [By[0],Hy1[0]],color = 'b', lw=3)
line9, = ax.plot([Cx[0],Lx[0]], [Cy[0],Ly[0]],color = 'm', lw=4)   
line10, = ax.plot([Dx[0],Mx[0]], [Dy[0],My[0]],color = 'm', lw=4)  
line11, = ax.plot([Ox[0],Qx[0]], [Oy[0],Qy[0]],color = 'm', lw=4)  
line12, = ax.plot([Fx[0],Gx1[0]], [Fy[0],Gy1[0]],color = 'r', lw=3)   
line13, = ax.plot([Gx1[0],Tx[0]], [Gy1[0],Ty[0]],color = 'r', lw=3)   
line14, = ax.plot([Tx[0],Ex[0]], [Ty[0],Ey[0]],color = 'r', lw=3)   
line15, = ax.plot([Dx[0],Hx[0]], [Dy[0],Hy[0]],color = 'b', lw=3)  
line16, = ax.plot([Hx[0],Gx1[0]], [Hy[0],Gy1[0]],color = 'k', lw=3.5,marker='o')  
line17, = ax.plot([Hx[0],kkx[0]],[Hy[0],kky[0]],color = 'r', lw=3,marker='o')

ax.set_xlabel('Span Panjang Bed (mm)')
ax.set_ylabel('Tinggi Angkat (mm)')
ax.set_aspect('equal')

i= range(len(Ax))

st.pyplot(fig)
# matplotlib.pyplot.close()
## Create Fungsi Animasi


Z = st.sidebar.slider("Number of Ensembles", min_value=10, max_value=50, value=20, step=10)

def animate(i):
    line1.set_data([Ax[i],Cx[i]], [Ay[i],Cy[i]])    
    line2.set_data([Cx[i],Ex[i]], [Cy[i],Ey[i]])  
    line3.set_data([Ax[i],Ex[i]], [Ay[i],Ey[i]])  
    line4.set_data([Bx[i],Dx[i]], [By[i],Dy[i]])   
    line5.set_data([Dx[i],Fx[i]], [Dy[i],Fy[i]])    
    line6.set_data([Gx[i],Fx[i]], [Gy[i],Fy[i]])   
    line7.set_data([Bx[i],Fx[i]], [By[i],Fy[i]])   
    line8.set_data([Bx[i],Hx1[i]], [By[i],Hy1[i]])
    line9.set_data([Cx[i],Lx[i]], [Cy[i],Ly[i]])   
    line10.set_data([Dx[i],Mx[i]], [Dy[i],My[i]])  
    line11 .set_data([Ox[i],Qx[i]], [Oy[i],Qy[i]])  
    line12.set_data([Fx[i],Gx1[i]], [Fy[i],Gy1[i]])   
    line13.set_data([Gx1[i],Tx[i]], [Gy1[i],Ty[i]])   
    line14.set_data([Tx[i],Ex[i]], [Ty[i],Ey[i]])   
    line15.set_data([Dx[i],Hx[i]], [Dy[i],Hy[i]])  
    line16.set_data([Hx[i],Gx1[i]], [Hy[i],Gy1[i]])  
    line17.set_data([Hx[i],kkx[i]],[Hy[i],kky[i]]) 
    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16, line17           
    
# Create the animation
ani = animation.FuncAnimation(fig, animate, frames=np.array(i), blit=True)

# Save the animation to a temporary file
with tempfile.NamedTemporaryFile(delete=False, suffix=".gif") as tmpfile:
    ani.save(tmpfile.name, writer='pillow')
    tmpfile_path = tmpfile.name

# Display the animation in Streamlit
st.title("Animasi Gerakan Tempat Tidur")
st.image(tmpfile_path)

st.write("## Evaluasi Pencapaian Target Gerakan ")
st.write("### Nilai Target ")

# Display the repayments.
col1, col2, col3, col4 = st.columns(4)
target_mini = col1.number_input("Ketinggian Terendah(mm)", min_value=280, value=290)
target_maksi = col2.number_input("Ketinggian Maksimum(mm)", min_value=600, value=630)
target_langkah = col3.number_input("Langkah Motor (mm)",value=200)
target_gaya_motor = col4.number_input("Gaya Motor Standar (N)",value=6000)

tmini = round(tinggi_angkat[0],0)
tmaksi = round(tinggi_angkat[pembagi],0)
stroke1 = round(stroke[pembagi],0)
Gaya_Maksi = round(max(R),0)

t1=100*(target_mini-tmini)/target_mini
t1=round(t1,0)
t2=100*(target_maksi-tmaksi)/target_maksi
t2=round(t2,0)
t3=100*(target_langkah-stroke1)/target_langkah
t3=round(t3,0)
t4=100*(target_gaya_motor-Gaya_Maksi)/target_gaya_motor
t4=round(t4,0)

st.write("### Pencapaian Target Gerakan ")
col1, col2, col3= st.columns(3)
target_mini1 = col1.number_input("Target Ketinggian Terendah(mm)", value=290)
target_maksi1 = col1.number_input("Target Ketinggi Maksimum (mm)", value=630)
target_langkah1 = col1.number_input("Target Langkah Motor (mm)",value=200)
target_gaya_motor1 = col1.number_input("Target Gaya Motor Standar (N)",value=6000)
hasil1 = col2.number_input("Ketinggian Minimum Tercapai (mm)",  value=tmini)
hasil2= col2.number_input("Ketinggian Maksimum Tercapai (mm)", value=tmaksi)
hasil3 = col2.number_input("Langkah Motor Maksimum(mm)",value=stroke1)
hasil4 = col2.number_input("Gaya Motor Maksimum (mm)",value=Gaya_Maksi)
Beda1 = col3.number_input("Beda Ketinggian Mini (%)",  value=t1)
Beda2 = col3.number_input("Beda Ketinggian Maksi (%)", value=t2)
Beda3 = col3.number_input("Beda Langkah Motor (%)",value=t3)
Beda3 = col3.number_input("Beda Gaya Motor  (%)",value=t4)

