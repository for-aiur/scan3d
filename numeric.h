#ifndef NUMERIC_H
#define NUMERIC_H

bool Householder(double **M, int Z, int S);

bool Solve3x3LGS(double A[][3], double *y, double x[], double Eps);

#endif /* NUMERIC_H */