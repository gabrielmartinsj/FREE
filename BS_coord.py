import matplotlib.pyplot as plt
import numpy as np

def base_coordinates(number_of_BS, maxDist):
    # Isso pode ser feito de uma forma muito mais inteligente com certeza
    base_xy = np.array([[0,0]]) + np.array([maxDist,maxDist]) + 10
    if number_of_BS == 1:
        return base_xy
    # Switch
    if number_of_BS == 2:
        base_xy = np.array([[-0.5, 0],
                            [ 0.5, 0]])*maxDist + np.array([maxDist,maxDist]) + 10

    elif number_of_BS == 3:
        base_xy = np.array([[-0.464101615137754587054892683011 , -0.267949192431122706472553658494],
                            [ 0.464101615137754587054892683011 , -0.267949192431122706472553658494],
                            [ 0.000000000000000000000000000000 ,  0.535898384862245412945107316988]])*maxDist+ np.array([maxDist,maxDist]) + 10
        
    elif number_of_BS == 4:
        base_xy = np.array([[-0.414213562373095048801688724210,  -0.414213562373095048801688724210],
                            [ 0.414213562373095048801688724210,  -0.414213562373095048801688724210],
                            [-0.414213562373095048801688724210,   0.414213562373095048801688724210],
                            [ 0.414213562373095048801688724210,   0.414213562373095048801688724210]])*maxDist + np.array([maxDist,maxDist]) + 10
    elif number_of_BS == 5:
        base_xy = np.array([   [-0.370191908158750137702237641058,  -0.509525449494428810513706911251],
                               [ 0.370191908158750137702237641058,  -0.509525449494428810513706911251],
                               [-0.598983089761037227177173011864,   0.194621403573803879364825731779],
                               [ 0.598983089761037227177173011864,   0.194621403573803879364825731779],
                               [ 0.000000000000000000000000000000,   0.629808091841249862297762358942]])*maxDist + np.array([maxDist,maxDist]) + 10
    elif number_of_BS == 6:
        base_xy = np.array([   [-0.333333333333333333333333333333,  -0.577350269189625764509148780502],
                               [ 0.333333333333333333333333333333,  -0.577350269189625764509148780502],
                               [-0.666666666666666666666666666667,   0.000000000000000000000000000000],
                               [ 0.666666666666666666666666666667,   0.000000000000000000000000000000],
                               [-0.333333333333333333333333333333,   0.577350269189625764509148780502],
                               [ 0.333333333333333333333333333333,   0.577350269189625764509148780502]])*maxDist + np.array([maxDist,maxDist]) + 10
    elif number_of_BS == 7:
        base_xy = np.array([   [-0.333333333333333333333333333333,  -0.577350269189625764509148780502],
                               [ 0.333333333333333333333333333333,  -0.577350269189625764509148780502],
                               [-0.666666666666666666666666666667,   0.000000000000000000000000000000],
                               [ 0.000000000000000000000000000000,   0.000000000000000000000000000000],
                               [ 0.666666666666666666666666666667,   0.000000000000000000000000000000],
                               [-0.333333333333333333333333333333,   0.577350269189625764509148780502],
                               [ 0.333333333333333333333333333333,   0.577350269189625764509148780502]])*maxDist + np.array([maxDist,maxDist]) + 10
    elif number_of_BS == 8:
        base_xy = np.array([   [-0.302593388348611302909204224933, -0.628341645367213738512227388956],
                               [ 0.302593388348611302909204224933, -0.628341645367213738512227388956],
                               [-0.679921171839088240043878874469, -0.155187570571975671990838057814],
                               [ 0.679921171839088240043878874469, -0.155187570571975671990838057814],
                               [ 0.000000000000000000000000000000,  0.000000000000000000000000000000],
                               [-0.545254445070410775447749861103,  0.434825910113495061957667559237],
                               [ 0.545254445070410775447749861103,  0.434825910113495061957667559237],
                               [ 0.000000000000000000000000000000,  0.697406611651388697090795775067]])*maxDist + np.array([maxDist,maxDist]) + 10
    return base_xy