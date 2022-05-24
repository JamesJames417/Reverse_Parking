def Car_Kinematics_Model(x,y,phi,theta):
    import math
    phi=phi/180*math.pi
    theta=theta/180*math.pi
    x=x+math.cos(phi+theta)+math.sin(phi)*math.sin(theta)
    y=y+math.sin(phi+theta)-math.sin(phi)*math.sin(theta)
    phi=phi-math.asin(2*math.sin(theta)/4) # 4為車長
    phi=phi/math.pi*180
    return x,y,phi

def Fuzzy_Controller(V1,V2):
    import numpy
    # 定義linquist variable(delta x、delta phi、theta)的universe of discourse
    universe_delta_x=numpy.arange(-10,10,0.1) # delta x
    universe_delta_phi=numpy.arange(-90,90,0.1) # delta phi
    universe_theta=numpy.arange(-16,16,0.1) # theta

    import skfuzzy.control
    # 定義linquist variable (delta x、delta phi、theta)
    delta_x=skfuzzy.control.Antecedent(universe_delta_x,'delta_x')
    delta_phi=skfuzzy.control.Antecedent(universe_delta_phi,'delta_phi')
    theta=skfuzzy.control.Consequent(universe_theta,'theta')

    # 定義delta x的linquist value及其membership function
    delta_x['A11']=skfuzzy.trapmf(universe_delta_x,[-10,-10,-6.5,-4.5])
    delta_x['A11.5'] = skfuzzy.trimf(universe_delta_x, [-5, -2.5, 0])
    delta_x['A12']=skfuzzy.trimf(universe_delta_x,[-2,0,2])
    delta_x['A12.5'] = skfuzzy.trimf(universe_delta_x, [0, 2.5, 5])
    delta_x['A13']=skfuzzy.trapmf(universe_delta_x,[4.5,6.5,10,10])



    # 定義delta phi的linquist value及其membership function
    delta_phi['A21']=skfuzzy.trapmf(universe_delta_phi,[-90,-90,-65,-45])
    delta_phi['A21.5'] = skfuzzy.trimf(universe_delta_phi, [-70, -35, 0])
    delta_phi['A22']=skfuzzy.trimf(universe_delta_phi,[-15,0,15])
    delta_phi['A22.5'] = skfuzzy.trimf(universe_delta_phi, [0, 35, 70])
    delta_phi['A23']=skfuzzy.trapmf(universe_delta_phi,[45,65,90,90])

    # 定義theta的linquist value及其membership function
    theta['Y1']=skfuzzy.trimf(universe_theta,[-16,-12,-7])
    theta['Y1.5'] = skfuzzy.trimf(universe_theta, [-8, -4, 0])
    theta['Y2']=skfuzzy.trimf(universe_theta,[-2.5,0,2.5])
    theta['Y2.5'] = skfuzzy.trimf(universe_theta, [0, 4, 8])
    theta['Y3']=skfuzzy.trimf(universe_theta,[7,12,16])

    # 解模糊化方法
    theta.defuzzify_method='centroid' # 重心法

    import matplotlib.pyplot as plt
    # 繪membership funcion圖
    # delta_x.view()
    # delta_phi.view()
    #theta.view()
    # plt.show()

    # rule base
    rule1 = skfuzzy.control.Rule(antecedent=((delta_x['A11'] & delta_phi['A22']) | (delta_x['A11'] & delta_phi['A22.5']) | (delta_x['A11'] & delta_phi['A23']) | (delta_x['A11.5'] & delta_phi['A22.5']) | (delta_x['A11.5'] & delta_phi['A23']) | (delta_x['A12'] & delta_phi['A22.5'])| (delta_x['A12'] & delta_phi['A23'])),consequent=theta['Y3'], label='Y3')
    rule2 = skfuzzy.control.Rule(antecedent=((delta_x['A11.5'] & delta_phi['A22']) | (delta_x['A12.5'] & delta_phi['A22.5'])|(delta_x['A12.5'] & delta_phi['A23'])), consequent=theta['Y2.5'],label='Y2.5')
    rule3 = skfuzzy.control.Rule(antecedent=((delta_x['A11'] & delta_phi['A21'])|(delta_x['A11'] & delta_phi['A21.5'])|(delta_x['A12'] & delta_phi['A22'])|(delta_x['A13'] & delta_phi['A22.5'])|(delta_x['A13'] & delta_phi['A23'])), consequent=theta['Y2'], label='Y2')
    rule4 = skfuzzy.control.Rule(antecedent=((delta_x['A11.5'] & delta_phi['A21']) | (delta_x['A11.5'] & delta_phi['A21.5'])| (delta_x['A12.5'] & delta_phi['A22'])),consequent=theta['Y1.5'], label='Y1.5')
    rule5 = skfuzzy.control.Rule(antecedent=((delta_x['A12'] & delta_phi['A21']) | (delta_x['A12'] & delta_phi['A21.5']) | (delta_x['A12.5'] & delta_phi['A21']) | (delta_x['A12.5'] & delta_phi['A21.5']) | (delta_x['A13'] & delta_phi['A21']) | (delta_x['A13'] & delta_phi['A21.5'])| (delta_x['A13'] & delta_phi['A22'])), consequent=theta['Y1'], label='Y1')


    system=skfuzzy.control.ControlSystem(rules=[rule1, rule2,rule3, rule4, rule5])
    sim=skfuzzy.control.ControlSystemSimulation(system)
    sim.input['delta_x']=V1
    sim.input['delta_phi']=V2
    sim.compute()
    # theta.view(sim=sim)
    # plt.show()
    return sim.output['theta']

x=10 # x初始位置
y=600 # y初始位置
phi=56 # phi初始位置

x_pos=[0]*200
y_pos=[0]*200
for t in range(0,200,1):
    theta=Fuzzy_Controller(x-0,phi-90)
    x,y,phi=Car_Kinematics_Model(x,y,phi,theta)
    x_pos[t]=x
    y_pos[t]=y

import matplotlib.pyplot as plt
plt.plot(x_pos,y_pos)
plt.show()

