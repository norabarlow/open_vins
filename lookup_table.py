# https://web.eecs.utk.edu/~azh/blog/cosine.html
from math import sin, cos, pi

def cos_table(f, PRECISION, NAME):
    f.write("static double %s[] = {\n" % NAME)
    j = 0
    p = 0.0
    while True:
        f.write("{:.20f}, ".format(cos(p)))
        j += 1
        p += PRECISION
        if p > 2*pi:
            break
    f.write("1.0 };\n")
    f.write("const int %s_size = %d;\n" % (NAME, j+1))

def sin_table(f, PRECISION, NAME):
    f.write("static double %s[] = {\n" % NAME)
    j = 0
    p = 0.0
    while True:
        f.write("{:.20f}, ".format(sin(p)))
        j += 1
        p += PRECISION
        if p > 2*pi:
            break
    f.write("1.0 };\n")
    f.write("const int %s_size = %d;\n" % (NAME, j+1))

if __name__ == '__main__':
    cos_table(open("floatx/src/costable_1.h", "w"), 1.0, "costable_1")
    cos_table(open("floatx/src/costable_0_1.h", "w"), 0.1, "costable_0_1")
    cos_table(open("floatx/src/costable_0_01.h", "w"), 0.01, "costable_0_01")
    cos_table(open("floatx/src/costable_0_001.h", "w"), 0.001, "costable_0_001")
    cos_table(open("floatx/src/costable_0_0001.h", "w"), 0.0001, "costable_0_0001")

    sin_table(open("floatx/src/sintable_1.h", "w"), 1.0, "sintable_1")
    sin_table(open("floatx/src/sintable_0_1.h", "w"), 0.1, "sintable_0_1")
    sin_table(open("floatx/src/sintable_0_01.h", "w"), 0.01, "sintable_0_01")
    sin_table(open("floatx/src/sintable_0_001.h", "w"), 0.001, "sintable_0_001")
    sin_table(open("floatx/src/sintable_0_0001.h", "w"), 0.0001, "sintable_0_0001")
