import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

DER_ADR_TTN_1500  = np.zeros(9)
DER_ADR_TTN_2500  = np.zeros(9)
DER_ADR_TTN_3500  = np.zeros(9)
DER_ADR_TTN_4500  = np.zeros(9)

for i in range(9):
    ADR_TTN1 = pd.read_csv("ADR-TTN/1000-1200-{}-20-1500-1-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    ADR_TTN2 = pd.read_csv("ADR-TTN/1000-1200-{}-20-2500-1-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    ADR_TTN3 = pd.read_csv("ADR-TTN/1000-1200-{}-20-3500-1-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    ADR_TTN4 = pd.read_csv("ADR-TTN/1000-1200-{}-20-4500-1-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    DER_ADR_TTN_1500[i] += (np.average(ADR_TTN1[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    DER_ADR_TTN_2500[i] += (np.average(ADR_TTN2[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    DER_ADR_TTN_3500[i] += (np.average(ADR_TTN3[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    DER_ADR_TTN_4500[i] += (np.average(ADR_TTN4[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))

plt.plot(range(9), DER_ADR_TTN_1500, label='1500 m')
plt.plot(range(9), DER_ADR_TTN_2500, label='2500 m')
plt.plot(range(9), DER_ADR_TTN_3500, label='3500 m')
plt.plot(range(9), DER_ADR_TTN_4500, label='4500 m')
plt.legend()
plt.ylabel("DER")
plt.axis([-0.5,8.5,0.55,1])
plt.xlabel("repetições")
plt.show()

DER_ADR_TTN_1500  = np.zeros(8)
DER_ADR_TTN_2500  = np.zeros(8)
DER_ADR_TTN_3500  = np.zeros(8)
DER_ADR_TTN_4500  = np.zeros(8)
for i in range(1,9):
    ADR_TTN1 = pd.read_csv("old/ADR-TTN/multiGW/200-1200-0-20-1500-{}-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    ADR_TTN2 = pd.read_csv("old/ADR-TTN/multiGW/200-1200-0-20-2500-{}-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    ADR_TTN3 = pd.read_csv("old/ADR-TTN/multiGW/200-1200-0-20-3500-{}-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    ADR_TTN4 = pd.read_csv("old/ADR-TTN/multiGW/200-1200-0-20-4500-{}-1-ADR-TTN-10-0.8-1/scalars-0.csv".format(i))
    DER_ADR_TTN_1500[i-1] += (np.average(ADR_TTN1[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    DER_ADR_TTN_2500[i-1] += (np.average(ADR_TTN2[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    DER_ADR_TTN_3500[i-1] += (np.average(ADR_TTN3[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    DER_ADR_TTN_4500[i-1] += (np.average(ADR_TTN4[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))

plt.plot(range(1,9), DER_ADR_TTN_1500, label='1500 m')
plt.plot(range(1,9), DER_ADR_TTN_2500, label='2500 m')
plt.plot(range(1,9), DER_ADR_TTN_3500, label='3500 m')
plt.plot(range(1,9), DER_ADR_TTN_4500, label='4500 m')
plt.legend()
plt.ylabel("DER")
plt.axis([0.5,8.5,0.55,1])
plt.xlabel("GW")
plt.show()