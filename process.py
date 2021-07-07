import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import LoRaWAN
import time
rep = 1
margins = False
replicas = True
BSs = True

if margins:
    #NO ADR
    start = time.time()
    argv = [200,1200,0,20,1500,1,0,1,0]
    for i in range(rep):
        argv[-1]=i
        LoRaWAN.main(argv)

    # ADR
    argv = [200,1200,0,20,1500,1,1,"ADR-TTN",10,0.8,1,0]
    ADRs = ["ADR-TTN", "ADR+"]
    for ADR in ADRs:
        argv[-5] = ADR
        for i in range(5,45,5):
            argv[-4] = i
            for j in range(rep):
                argv[-1] = j
                LoRaWAN.main(argv)
    finish = time.time()
    print("ELAPSED TIME: {}".format(finish-start))
    DER_ADRp = np.zeros(8)
    DER_ADR_TTN = np.zeros(8)
    DER_none = 0

    DER_ADR_TTN_f = np.zeros(len(range(5,45,5)))
    DER_ADRp_f = np.zeros(len(range(5,45,5)))
    for l in range(rep):
        none = pd.read_csv("none/200-1200-20-1500-1-0-1/scalars-{}.csv".format(l))
        DER_none += none[["node_{}".format(k) for k in range(199)]].to_numpy()[0].sum()/200
        print(DER_none)
        for i,j in enumerate(range(5,45,5)):
            ADR_TTN = pd.read_csv("ADR-TTN/200-1200-20-1500-1-1-ADR-TTN-{}-0.8-1/scalars-{}.csv".format(j,l))
            ADRp = pd.read_csv("ADR+/200-1200-20-1500-1-1-ADR+-{}-0.8-1/scalars-{}.csv".format(j,l))
            DER_ADRp[i] += (np.average(ADRp[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
            DER_ADR_TTN[i] += (np.average(ADR_TTN[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))

    DER_none /= rep
    print(DER_none)
    DER_ADR_TTN /= rep
    DER_ADRp /= rep
    plt.plot(range(5,45,5), [DER_none]*len(range(5,45,5)),label='No ADR')
    plt.plot(range(5,45,5), DER_ADR_TTN,label='ADR-TTN')
    plt.plot(range(5,45,5), DER_ADRp,label='ADR+')
    plt.legend()
    plt.ylabel("DER")
    plt.axis([4.5,40.5,0,1])
    plt.xlabel("margin_db")
    plt.show()

if replicas:
    # start = time.time()
    # argv = [200,1200,0,20,1500,1,0,1,0]
    # for i in range(rep):
    #     for j in range(9):
    #         for dist in range(2500,5500,1000):
    #             argv[4] = dist
    #             argv[-1]=i
    #             argv[2] = j
    #             LoRaWAN.main(argv)

    # ADR
    start=time.time()
    argv = [1000,1200,0,20,1500,1,1,"ADR-TTN",10,0.8,1,0]
    ADRs = ["ADR-TTN"]#, "ADR+"]
    for ADR in ADRs:
        argv[-5] = ADR
        for i in range(0,9):
            argv[2] = i
            for j in range(rep):
                argv[-1] = j
                for dist in range(1500,6500,1000):
                    argv[4] = dist
                    LoRaWAN.main(argv)
    finish = time.time()
    print("ELAPSED TIME: {}".format(finish-start))
    DER_ADRp = np.zeros(9)
    DER_ADR_TTN = np.zeros(9)
    DER_none = 0

    # DER_ADR_TTN_f = np.zeros(len(9))
    # DER_ADRp_f = np.zeros(len(9))
    # for l in range(rep):
    #     # none = pd.read_csv("none/200-1200-{}-20-1500-1-0-1/scalars-{}.csv".format(l))
    #     # DER_none += none[["node_{}".format(k) for k in range(199)]].to_numpy()[0].sum()/200
    #     print(DER_none)
    #     for i,j in enumerate(range(9)):
    #         ADR_TTN = pd.read_csv("ADR-TTN/200-1200-{}-20-1500-1-1-ADR-TTN-10-0.8-1/scalars-{}.csv".format(j,l))
    #         ADRp = pd.read_csv("ADR+/200-1200-{}-20-1500-1-1-ADR+-10-0.8-1/scalars-{}.csv".format(j,l))
    #         DER_ADRp[i] += (np.average(ADRp[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    #         DER_ADR_TTN[i] += (np.average(ADR_TTN[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))

    # # DER_none /= rep
    # # print(DER_none)
    # DER_ADR_TTN /= rep
    # DER_ADRp /= rep
    # # plt.plot(range(5,45,5), [DER_none]*len(range(5,45,5)),label='No ADR')
    # plt.plot(range(5,45,5), DER_ADR_TTN,label='ADR-TTN')
    # # plt.plot(range(5,45,5), DER_ADRp,label='ADR+')
    # plt.legend()
    # plt.ylabel("DER")
    # plt.axis([0,9,0,1])
    # plt.xlabel("replicas")
    # plt.show()

if BSs:
    start = time.time()
    # argv = [200,1200,0,20,1500,1,0,1,0]
    # for i in range(rep):
    #     for j in range(9):
    #         argv[-1]=i
    #         argv[2] = j
    #         LoRaWAN.main(argv)

    # ADR
    argv = [1000,1200,0,20,1500,1,1,"ADR-TTN",10,0.8,1,0]
    ADRs = ["ADR-TTN"]#, "ADR+"]
    for dist in range(1500,6500,1000):
        argv[4] = dist
        for i in range(1,9):
            argv[5] = i
            for j in range(rep):
                argv[-1] = j
                LoRaWAN.main(argv)
    finish = time.time()
    print("ELAPSED TIME: {}".format(finish-start))

    # for l in range(rep):
    #     # none = pd.read_csv("none/200-1200-{}-20-1500-1-0-1/scalars-{}.csv".format(l))
    #     # DER_none += none[["node_{}".format(k) for k in range(199)]].to_numpy()[0].sum()/200
    #     print(DER_none)
    #     for i,j in enumerate(range(1,9)):
    #         ADR_TTN = pd.read_csv("ADR-TTN/200-1200-{}-20-1500-1-1-ADR-TTN-10-0.8-1/scalars-{}.csv".format(j,l))
    #         ADRp = pd.read_csv("ADR+/200-1200-{}-20-1500-1-1-ADR+-10-0.8-1/scalars-{}.csv".format(j,l))
    #         DER_ADRp[i] += (np.average(ADRp[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))
    #         DER_ADR_TTN[i] += (np.average(ADR_TTN[["node_{}".format(k) for k in range(199)]].to_numpy()[0]))

    # # DER_none /= rep
    # # print(DER_none)
    # DER_ADR_TTN /= rep
    # DER_ADRp /= rep
    # # plt.plot(range(5,45,5), [DER_none]*len(range(5,45,5)),label='No ADR')
    # plt.plot(range(1,9), DER_ADR_TTN,label='ADR-TTN')
    # # plt.plot(range(5,45,5), DER_ADRp,label='ADR+')
    # plt.legend()
    # plt.ylabel("DER")
    # plt.axis([0,9,0,1])
    # plt.xlabel("# GWs")
    # plt.show()