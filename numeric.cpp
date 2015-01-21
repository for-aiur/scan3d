#include <cmath>

bool Householder(double **M, int Z, int S)
{
   register int i, j, k;
   register double *mi, *mj;
   double c, f, q;
   double e = 1.0E-12;

   for( k=0; k<S; k++ )
   {
      c = 0;
      for( i=k; i<Z; i++ ){
        f = M[i][k];
        c += f*f;
      }
      if( M[k][k]<0 ) c = sqrt( c ); else c = -sqrt( c );
      if( fabs(c)<e ) return false;
      q = 1/((c-M[k][k])*c);
      M[k][k] -= c;
      for( j=k+1; j<=S; j++ ){
         f = 0;
         for( i=k; i<Z; i++ ){
           mi = M[i];
           f += mi[k]*mi[j];
         }
         f *= q;
         for( i=k; i<Z; i++ ){
           mi = M[i];
           mi[j] -= f*mi[k];
         }
      }
      M[k][k]=c;
      for( i=k+1; i<Z; i++ ){
        M[i][k]=0;
      }
   }

   // Rückwärts einsetzen
   for( i=S-1; i>=0; i-- ){
      mi = M[i];
      for( j=S; j>=i; j-- ){
        mi[j] /= mi[i];
      }
      for( j=i-1; j>=0; j-- ){
        mj = M[j];
        for( k=S; k>=i; k-- ){
          mj[k] -= mi[k]*mj[i];
        }
      }
   }
   return true;
}

bool Solve3x3LGS(double A[][3], double *y, double x[], double Eps)
{
	// determinant
    const double d =  ( A[0][0]*A[2][2]- A[2][0]*A[0][2] ) * A[1][1]
		            + ( A[0][2]*A[2][1]- A[2][2]*A[0][1] ) * A[1][0]
					+ ( A[0][1]*A[2][0]- A[2][1]*A[0][0] ) * A[1][2];

    if (fabs(d) < Eps)
        return false;

	x[0] = ( ( A[1][1]*A[2][2]-A[2][1]*A[1][2])*y[0]+
		     ( A[0][2]*A[2][1]-A[2][2]*A[0][1])*y[1]+
		     ( A[0][1]*A[1][2]-A[1][1]*A[0][2])*y[2] ) / d;

	x[1] = ( ( A[1][2]*A[2][0]-A[2][2]*A[1][0])*y[0]+
		     ( A[0][0]*A[2][2]-A[2][0]*A[0][2])*y[1]+
		     ( A[0][2]*A[1][0]-A[1][2]*A[0][0])*y[2] ) / d;

	x[2] = ( ( A[1][0]*A[2][1]-A[2][0]*A[1][1])*y[0]+
		     ( A[0][1]*A[2][0]-A[2][1]*A[0][0])*y[1]+
		     ( A[0][0]*A[1][1]-A[1][0]*A[0][1])*y[2] ) / d;

	return true;
}