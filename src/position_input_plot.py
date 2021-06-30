import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import FontProperties

font = FontProperties(fname=r"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",size=12)
x=1
u=0

if(x==1):
	f0= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/x_planed_outsearch.txt", 'r', True)
	#f1= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/x_planed_outsearch.txt", 'r', True)
	#f2= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/x_planed_outsearch.txt", 'r', True)
	#f3= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/x_planed_outsearch.txt", 'r', True)
	fu0= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/priordata/u_planed_outsearch.txt", 'r', True)
	#fu1= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/u_planed_outsearch.txt", 'r', True)
	#fu2= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/u_planed_outsearch.txt", 'r', True)
	#fu3= open("/home/cby/drake_learning/src/drake_learning_show_multistair/src/searchdata/u_planed_outsearch.txt", 'r', True)
list_x0_0 = []          
list_x0_1 = []          
list_x0_2 = []          
list_x0_3 = []          
list_x0_4 = []          
list_x0_5 = []          
list_x0_6 = []      
list_x0_7 = []         
list_x0_8 = []         
list_x0_9 = []          
list_x0_10 = []          
list_x0_11 = []          
list_x0_12 = []          
list_x0_13 = []          
list_x0_14 = []          
list_x0_15 = []          
list_x0_16 = []          
list_x0_17 = []          
list_x0_18 = []      
list_x0_19 = []         
list_x0_20 = []         
list_x0_21 = []          
list_x0_22 = []          
list_x0_23 = []       
list_x0_24 = []         
list_x0_25 = []         
list_x0_26 = []          
list_x0_27 = []          
list_x0_28 = []   
list_u0 = []          
list_u1 = []          
list_u2 = []          
list_u3 = []        
list_u4 = []          
list_u5 = []        


for num in range(0,19): #
	list_x0_0.append(round(float(f0.readline().rstrip()),3))
	list_x0_1.append(round(float(f0.readline().rstrip()),3))
	list_x0_2.append(round(float(f0.readline().rstrip()),3))
	list_x0_3.append(round(float(f0.readline().rstrip()),3))
	list_x0_4.append(round(float(f0.readline().rstrip()),3))
	list_x0_5.append(round(float(f0.readline().rstrip()),3))
	list_x0_6.append(round(float(f0.readline().rstrip()),3))
	list_x0_7.append(round(float(f0.readline().rstrip()),3))
	list_x0_8.append(round(float(f0.readline().rstrip()),3))
	list_x0_9.append(round(float(f0.readline().rstrip()),3))
	list_x0_10.append(round(float(f0.readline().rstrip()),3))
	list_x0_11.append(round(float(f0.readline().rstrip()),3))
	list_x0_12.append(round(float(f0.readline().rstrip()),3))
	list_x0_13.append(round(float(f0.readline().rstrip()),3))
	list_x0_14.append(round(float(f0.readline().rstrip()),3))
	list_x0_15.append(round(float(f0.readline().rstrip()),3))
	list_x0_16.append(round(float(f0.readline().rstrip()),3))
	list_x0_17.append(round(float(f0.readline().rstrip()),3))
	list_x0_18.append(round(float(f0.readline().rstrip()),3))
	list_x0_19.append(round(float(f0.readline().rstrip()),3))
	list_x0_20.append(round(float(f0.readline().rstrip()),3))
	list_x0_21.append(round(float(f0.readline().rstrip()),3))
	list_x0_22.append(round(float(f0.readline().rstrip()),3))
	list_x0_23.append(round(float(f0.readline().rstrip()),3))
	list_x0_24.append(round(float(f0.readline().rstrip()),3))
	list_x0_25.append(round(float(f0.readline().rstrip()),3))
	list_x0_26.append(round(float(f0.readline().rstrip()),3))
	list_x0_27.append(round(float(f0.readline().rstrip()),3))
	list_x0_28.append(round(float(f0.readline().rstrip()),3))
	list_u0.append(round(float(fu0.readline().rstrip()),3))
	list_u1.append(round(float(fu0.readline().rstrip()),3))
	list_u2.append(round(float(fu0.readline().rstrip()),3))
	list_u3.append(round(float(fu0.readline().rstrip()),3))
	list_u4.append(round(float(fu0.readline().rstrip()),3))
	list_u5.append(round(float(fu0.readline().rstrip()),3))

#关节位置
fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 18, 19)	
ax0=fig.add_subplot(221)#hip_roll
ax1=fig.add_subplot(222)#hip_yaw
ax2=fig.add_subplot(223)#hip_pitch
ax3=fig.add_subplot(224)#knee
ax0.set_title('驱动关节完整步态位置曲线(左右髋关节hip::roll)', fontproperties=font, fontsize=18)
ax1.set_title('驱动关节完整步态位置曲线(左右髋关节hip::yaw)', fontproperties=font, fontsize=18)
ax2.set_title('驱动关节完整步态位置曲线(左右髋关节hip::pitch)', fontproperties=font, fontsize=18)
ax3.set_title('驱动关节完整步态位置曲线(左右膝关节knee)', fontproperties=font, fontsize=18)

#plt.title('驱动关节完整步态力矩曲线(髋关节)', fontproperties=font)
#plt.title('驱动关节完整步态位置曲线(髋关节)', fontproperties=font)
#plt.xlabel("时间(s)",fontproperties=font)
#plt.ylabel("力矩(Nm)",fontproperties=font)
ax0.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax0.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax1.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax1.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax2.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax2.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax3.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax3.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
#ax0.set_xlabel("时间(timestep)",fontproperties=font)
#ax0.set_ylabel("力矩(Nm)",fontproperties=font)
#ax1.set_xlabel("时间(timestep)",fontproperties=font)
#ax1.set_ylabel("力矩(Nm)",fontproperties=font)
#ax2.set_xlabel("时间(timestep)",fontproperties=font)
#ax2.set_ylabel("力矩(Nm)",fontproperties=font)
#ax3.set_xlabel("时间(timestep)",fontproperties=font)
#ax3.set_ylabel("力矩(Nm)",fontproperties=font)

my_x_ticks = np.arange(0, 40, 2)#1.0hz
ax0.set_xticks(my_x_ticks)
ax1.set_xticks(my_x_ticks)
ax2.set_xticks(my_x_ticks)
ax3.set_xticks(my_x_ticks)
#my_y_ticks = np.linspace(-2, 2.2, 9)
#plt.yticks(my_y_ticks)
#plt.plot(times,list2,label='右髋关节',color='g',linewidth=2,linestyle='--')#position
#plt.plot(times,list3,label='左髋关节',color='b',linewidth=2,linestyle='--')#position
ax0.plot(times,list_x0_7,label='左髋关节roll',color='g',linewidth=2,linestyle='--')
ax0.plot(times,list_x0_8,label='右髋关节roll',color='b',linewidth=2,linestyle='--')
ax0.grid()#添加网格
ax0.legend(prop=font)#loc='lower right'loc='lower left', 

ax1.plot(times,list_x0_9,label='左髋关节yaw',color='g',linewidth=2,linestyle='--')
ax1.plot(times,list_x0_10,label='右髋关节yaw',color='b',linewidth=2,linestyle='--')
ax1.grid()#添加网格
ax1.legend(prop=font)#loc='lower right', 

ax2.plot(times,list_x0_11,label='左髋关节pitch',color='g',linewidth=2,linestyle='--')
ax2.plot(times,list_x0_12,label='右髋关节pitch',color='b',linewidth=2,linestyle='--')
ax2.grid()#添加网格
ax2.legend( prop=font)#loc='lower right',

ax3.plot(times,list_x0_13,label='左膝关节',color='g',linewidth=2,linestyle='--')#list_x0_3_u
ax3.plot(times,list_x0_14,label='右膝关节',color='b',linewidth=2,linestyle='--')#list_x1_3_u
ax3.grid()#添加网格
ax3.legend(prop=font)#loc='lower right', 
plt.tight_layout(3.5)
#plt.legend(loc='lower right', prop=font)
#plt.grid()#添加网格
plt.savefig("joint_position.jpeg",dpi = 600)
plt.show()



#基座四元数
fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 18, 19)	
ax0=fig.add_subplot(221)#hip_roll
ax1=fig.add_subplot(222)#hip_yaw
ax2=fig.add_subplot(223)#hip_pitch
ax3=fig.add_subplot(224)#knee


ax0.set_title('base_qw', fontproperties=font, fontsize=18)
ax1.set_title('base_qx', fontproperties=font, fontsize=18)
ax2.set_title('base_qy', fontproperties=font, fontsize=18)
ax3.set_title('base_qz', fontproperties=font, fontsize=18)

ax0.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax0.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax1.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax1.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax2.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax2.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax3.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax3.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)

my_x_ticks = np.arange(0, 40, 2)#1.0hz
ax0.set_xticks(my_x_ticks)
ax1.set_xticks(my_x_ticks)
ax2.set_xticks(my_x_ticks)
ax3.set_xticks(my_x_ticks)

#ax0.plot(times,list_x0_7,label='左髋关节roll',color='g',linewidth=2,linestyle='--')
ax0.plot(times,list_x0_0,label='base_qw',color='b',linewidth=2,linestyle='--')
ax0.grid()#添加网格
ax0.legend(prop=font)#loc='lower right'loc='lower left', 

ax1.plot(times,list_x0_1,label='base_qx',color='b',linewidth=2,linestyle='--')
ax1.grid()#添加网格
ax1.legend(prop=font)#loc='lower right', 

ax2.plot(times,list_x0_2,label='base_qy',color='g',linewidth=2,linestyle='--')
ax2.grid()#添加网格
ax2.legend( prop=font)#loc='lower right',

ax3.plot(times,list_x0_3,label='base_qz',color='b',linewidth=2,linestyle='--')#list_x1_3_u
ax3.grid()#添加网格
ax3.legend(prop=font)#loc='lower right', 
plt.tight_layout(3.5)
#plt.legend(loc='lower right', prop=font)
#plt.grid()#添加网格
plt.savefig("quaternion.jpeg",dpi = 600)
plt.show()

#基座位置
fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 18, 19)	
ax0=fig.add_subplot(221)#hip_roll
ax1=fig.add_subplot(222)#hip_yaw
ax2=fig.add_subplot(223)#hip_pitch

ax0.set_title('base_x', fontproperties=font, fontsize=18)
ax1.set_title('base_y', fontproperties=font, fontsize=18)
ax2.set_title('base_z', fontproperties=font, fontsize=18)

ax0.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax0.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax1.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax1.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax2.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax2.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)

my_x_ticks = np.arange(0, 40, 2)#1.0hz
ax0.set_xticks(my_x_ticks)
ax1.set_xticks(my_x_ticks)
ax2.set_xticks(my_x_ticks)

#ax0.plot(times,list_x0_7,label='左髋关节roll',color='g',linewidth=2,linestyle='--')
ax0.plot(times,list_x0_4,label='base_x',color='b',linewidth=2,linestyle='--')
ax0.grid()#添加网格
ax0.legend(prop=font)#loc='lower right'loc='lower left', 

ax1.plot(times,list_x0_5,label='base_y',color='b',linewidth=2,linestyle='--')
ax1.grid()#添加网格
ax1.legend(prop=font)#loc='lower right', 

ax2.plot(times,list_x0_6,label='base_z',color='g',linewidth=2,linestyle='--')
ax2.grid()#添加网格
ax2.legend( prop=font)#loc='lower right',

plt.tight_layout(3.5)
#plt.legend(loc='lower right', prop=font)
#plt.grid()#添加网格
plt.savefig("base_position.jpeg",dpi = 600)
plt.show()


#关节速度
fig=plt.figure(figsize=(20, 12))
times = np.linspace(0, 18, 19)	
ax0=fig.add_subplot(221)#hip_roll
ax1=fig.add_subplot(222)#hip_yaw
ax2=fig.add_subplot(223)#hip_pitch
ax3=fig.add_subplot(224)#knee
ax0.set_title('驱动关节完整步态速度曲线(左右髋关节hip::rolldot)', fontproperties=font, fontsize=18)
ax1.set_title('驱动关节完整步态速度曲线(左右髋关节hip::yawdot)', fontproperties=font, fontsize=18)
ax2.set_title('驱动关节完整步态速度曲线(左右髋关节hip::pitchdot)', fontproperties=font, fontsize=18)
ax3.set_title('驱动关节完整步态速度曲线(左右膝关节kneedot)', fontproperties=font, fontsize=18)

#plt.title('驱动关节完整步态力矩曲线(髋关节)', fontproperties=font)
#plt.title('驱动关节完整步态位置曲线(髋关节)', fontproperties=font)
#plt.xlabel("时间(s)",fontproperties=font)
#plt.ylabel("力矩(Nm)",fontproperties=font)
ax0.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax0.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)
ax1.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax1.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)
ax2.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax2.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)
ax3.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax3.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)

my_x_ticks = np.arange(0, 40, 2)#1.0hz
ax0.set_xticks(my_x_ticks)
ax1.set_xticks(my_x_ticks)
ax2.set_xticks(my_x_ticks)
ax3.set_xticks(my_x_ticks)
#my_y_ticks = np.linspace(-2, 2.2, 9)
#plt.yticks(my_y_ticks)
#plt.plot(times,list2,label='右髋关节',color='g',linewidth=2,linestyle='--')#position
#plt.plot(times,list3,label='左髋关节',color='b',linewidth=2,linestyle='--')#position
ax0.plot(times,list_x0_21,label='左髋关节rolldot',color='g',linewidth=2,linestyle='--')
ax0.plot(times,list_x0_22,label='右髋关节rolldot',color='b',linewidth=2,linestyle='--')
ax0.grid()#添加网格
ax0.legend(prop=font)#loc='lower right'loc='lower left', 

ax1.plot(times,list_x0_23,label='左髋关节yawdot',color='g',linewidth=2,linestyle='--')
ax1.plot(times,list_x0_24,label='右髋关节yawdot',color='b',linewidth=2,linestyle='--')
ax1.grid()#添加网格
ax1.legend(prop=font)#loc='lower right', 

ax2.plot(times,list_x0_25,label='左髋关节pitchdot',color='g',linewidth=2,linestyle='--')
ax2.plot(times,list_x0_26,label='右髋关节pitchdot',color='b',linewidth=2,linestyle='--')
ax2.grid()#添加网格
ax2.legend( prop=font)#loc='lower right',

ax3.plot(times,list_x0_27,label='左膝关节dot',color='g',linewidth=2,linestyle='--')#list_x0_3_u
ax3.plot(times,list_x0_28,label='右膝关节dot',color='b',linewidth=2,linestyle='--')#list_x1_3_u
ax3.grid()#添加网格
ax3.legend(prop=font)#loc='lower right', 
plt.tight_layout(3.5)
#plt.legend(loc='lower right', prop=font)
#plt.grid()#添加网格
plt.savefig("joint_velocity.jpeg",dpi = 600)
plt.show()



#基座角速度
fig=plt.figure(figsize=(20, 12))
times = np.linspace(0, 18, 19)	
ax0=fig.add_subplot(221)
ax1=fig.add_subplot(222)
ax2=fig.add_subplot(223)

ax0.set_title('base_wx', fontproperties=font, fontsize=18)
ax1.set_title('base_wy', fontproperties=font, fontsize=18)
ax2.set_title('base_wz', fontproperties=font, fontsize=18)

ax0.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax0.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)
ax1.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax1.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)
ax2.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax2.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)
ax3.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax3.set_ylabel("速度(rad)",fontproperties=font,fontsize=15)

my_x_ticks = np.arange(0, 40, 2)#1.0hz
ax0.set_xticks(my_x_ticks)
ax1.set_xticks(my_x_ticks)
ax2.set_xticks(my_x_ticks)
ax3.set_xticks(my_x_ticks)

#ax0.plot(times,list_x0_7,label='左髋关节roll',color='g',linewidth=2,linestyle='--')
ax0.plot(times,list_x0_15,label='base_wx',color='b',linewidth=2,linestyle='--')
ax0.grid()#添加网格
ax0.legend(prop=font)#loc='lower right'loc='lower left', 

ax1.plot(times,list_x0_16,label='base_wy',color='b',linewidth=2,linestyle='--')
ax1.grid()#添加网格
ax1.legend(prop=font)#loc='lower right', 

ax2.plot(times,list_x0_17,label='base_wz',color='g',linewidth=2,linestyle='--')
ax2.grid()#添加网格
ax2.legend( prop=font)#loc='lower right',

plt.tight_layout(3.5)
#plt.legend(loc='lower right', prop=font)
#plt.grid()#添加网格
plt.savefig("quaterniondot.jpeg",dpi = 600)
plt.show()

#基座速度
fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 18, 19)	
ax0=fig.add_subplot(221)#hip_roll
ax1=fig.add_subplot(222)#hip_yaw
ax2=fig.add_subplot(223)#hip_pitch

ax0.set_title('base_vx', fontproperties=font, fontsize=18)
ax1.set_title('base_vy', fontproperties=font, fontsize=18)
ax2.set_title('base_vz', fontproperties=font, fontsize=18)

ax0.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax0.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax1.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax1.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)
ax2.set_xlabel("时间(timestep)",fontproperties=font,fontsize=15)
ax2.set_ylabel("位置(rad)",fontproperties=font,fontsize=15)

my_x_ticks = np.arange(0, 40, 2)#1.0hz
ax0.set_xticks(my_x_ticks)
ax1.set_xticks(my_x_ticks)
ax2.set_xticks(my_x_ticks)

#ax0.plot(times,list_x0_7,label='左髋关节roll',color='g',linewidth=2,linestyle='--')
ax0.plot(times,list_x0_18,label='base_x',color='b',linewidth=2,linestyle='--')
ax0.grid()#添加网格
ax0.legend(prop=font)#loc='lower right'loc='lower left', 

ax1.plot(times,list_x0_19,label='base_y',color='b',linewidth=2,linestyle='--')
ax1.grid()#添加网格
ax1.legend(prop=font)#loc='lower right', 

ax2.plot(times,list_x0_20,label='base_z',color='g',linewidth=2,linestyle='--')
ax2.grid()#添加网格
ax2.legend( prop=font)#loc='lower right',

plt.tight_layout(3.5)
#plt.legend(loc='lower right', prop=font)
#plt.grid()#添加网格
plt.savefig("base_velocity.jpeg",dpi = 600)
plt.show()
