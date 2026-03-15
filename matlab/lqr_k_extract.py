import datetime

def fclear():
    with open("../chassis/User/module/lqrcoef.h", "w") as f:
        f.write("/// @date " + datetime.datetime.now().strftime('%Y-%m-%d %H:%M') + "\n")
        f.write("\n#ifndef LQRCOEF_H\n#define LQRCOEF_H\n\n")

def extrect_coef(step):
    with open("lqr_k.m", "r") as f:
        ls = f.readlines()
    
        lk = []
        for l in ls[9:21]:
            n1 = l.index('L0.*') + 4
            n2 = l.index('t2.*') - 1
            n3 = l.index('t3.*') - 1
            n4 = l[n3+7:].index('.') + n3+7 - 2
        
            k1 = eval( l[n1:n2] )
            k2 = eval( l[n2:n3].replace("t2.*", "") )
            k3 = eval( l[n3:n4].replace("t3.*", "") )
            k4 = eval( l[n4:-3] )
        
            lk.append( [k1, k2, k3, k4] )

        head = "float lqr_coe{0}[12][4] = ".format(int(step)) + "{\n"
        body = "\t{{{0:.15f}, {1:.15f}, {2:.15f}, {3:.15f}}},\n"
        tail = "};\n\n"

        fout = open("../chassis/User/module/lqrcoef.h", "a")

        fout.write(head)
        for i in range(len(lk)):
            l = lk[i]
            if i == 11:
                fout.write( body[:-2].format(l[0], l[1], l[2], l[3]) )
            else:
                fout.write( body.format(l[0], l[1], l[2], l[3]) )
        fout.write(tail)

        fout.close()

def ftail():
    with open("../chassis/User/module/lqrcoef.h", "a") as f:
        f.write("#endif /* LQRCOEF_H */ \n")
