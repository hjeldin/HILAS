/**********************************************************
 * This file is generated by 20-sim ANSI-C Code Generator  
 *
 *  file:  src\common\xxinverse.cpp
 *  subm:  interaction_control
 *  model: ModelControl
 *  expmt: ModelControl
 *  date:  October 31, 2011
 *  time:  10:57:42 am
 *  user:  Campuslicentie 
 *  from:  Universiteit Twente
 *  build: 4.1.2.2
 **********************************************************/


/* This file implements the functions necessary for calculating
   the inverse of a matrix
*/

/* standard include files */
#include <stdlib.h>
#include <memory.h>
#include <math.h>

/* 20-sim include files */
#include "xxtypes.h"
#include "xxmatrix.h"


/* this function fills the vector with elements 0, 1, 2 .... rows
 */
void XXIndex (XXMatrix *v)
{
	/* we dont know if it is horizontal or vertical */
	XXInteger i, size;

	if( v->rows > v->columns )
		size = v->rows;
	else
		size = v->columns;
		
	for(i = 0; i < size; i++)
		v->mat[i] = i;
}


/* this function swaps the vector v elements according to the indices of p
 * the work array must be at least the size of v
 */
void XXPermute (XXMatrix *v, XXMatrix *p, XXDouble *workarray)
{
	/* we dont know if it is horizontal or vertical */
	XXInteger i, size;

	if( v->rows > v->columns )
		size = v->rows;
	else
		size = v->columns;

	for( i = 0; i < size; i++)
		workarray[i] = v->mat[(XXInteger)p->mat[i]];

	/* and copy the values back to the matrix v */
	memcpy(v->mat, workarray, size * sizeof(XXDouble));
}


/* solves the equations by forward substiting : Ly = b
 * and backward substiting : Ux = y
 */
void XXSubstitute (XXMatrix *dest, XXMatrix *v)
{
	XXDouble t, diag;
	XXInteger i, j, rows, columns;

	rows = dest->rows;
	columns = dest->columns;

	/* forward substitution.*/
	for( i = 0; i < rows; i++)
	{
		t = v->mat[i];
		for( j = 0; j < i; j++ )
			t -= dest->mat[i*columns+j] * v->mat[j];

		diag = dest->mat[i*columns+i];
		if( diag == 0.0 )
		{
			/* error */
			return;
		}
		t /= diag;
		v->mat[i] = t;
	}

	/* back substitution */
	for( i = rows-1 ; i >= 0; i-- )
	{
		t = v->mat[i];
		for( j = i+1 ; j < rows; j++)
			t -= dest->mat[i*columns+j] * v->mat[j];
		v->mat[i] = t;
	}
}


/* swap the row1-th row with the row2-th row 
*/
void XXSwapRows (XXMatrix *dest, XXInteger row1, XXInteger row2)
{
	XXDouble temp;
	XXInteger i, columns;

	columns = dest->columns;

	for( i = 0; i < columns; i++)
	{
		temp = dest->mat[row1 * columns + i];
		dest->mat[row1 * columns + i] = dest->mat[row2 * columns + i];
		dest->mat[row2 * columns + i] = temp;
	}
}


/* this function pivots the matrix, that means that zero elements are
 * avoided from the the diagonal if possible.
 * it returns an integer which can be used to adjust the sign of the
 * determinant of the matrix. This sign changes when a swap is being performed
 */
XXInteger XXPivot (XXMatrix *dest, XXMatrix *p, XXInteger i)
{
	XXInteger j, k, rows, columns, sgn;
	XXDouble t, mki;

	rows = dest->rows;
	columns = dest->columns;

	j = i;
	sgn = 1;
	t = 0;

	for( k = i; k < rows; k++)
	{
		mki = fabs(dest->mat[k*columns+i]);
		if( mki > t )
		{
			t = mki;
			j = k;
		}
	}

	if( j > i )
	{
		XXSwapRows(dest, i, j);	/* swap matrix rows		*/
		XXSwapRows(p, i, j);		/* swap indices			*/
		sgn = -sgn;				/* change sign of det	*/
	}
	return sgn;
}


/* this function creates a L and U CMatrix in one matrix.
 */
XXDouble XXDecompose (XXMatrix *dest, XXMatrix *p)
{
	XXDouble det, piv, mult;
	XXInteger i, j, k, rows, columns, sgn;

	rows = dest->rows;
	columns = dest->columns;

	det = 1.0;
	for( i = 0; i < rows; i++)
	{
		sgn = XXPivot(dest, p, i);
		piv = dest->mat[i*columns+i];
		det *= sgn * piv;

		if( det == 0.0 )
		{
			/* no solution is possible for this matrix.
			 * so inform the caller by returning this determinant.
			 */
			return 0.0;
		}

		for( j = i+1 ; j < rows; j++)
			dest->mat[i*columns+j] /= piv;

		for( j = i+1 ; j < rows ; j++)
		{
			mult = dest->mat[j*columns+i];
			if( mult != 0.0 )
			{
				for( k = i+1 ; k < rows; k++)
					dest->mat[j*columns+k] -= mult * dest->mat[i*columns+k];
			}
		}

	}
	return det;
}


/* this function returns the determinant of the matrix.
 * workarray must be at least 2 * dest->rows long
 */
XXDouble XXCrout1 (XXMatrix *dest, XXMatrix *v, XXDouble *workarray)
{
	XXMatrix p;
	XXDouble det;
	XXDouble *permwork;

	p.rows = dest->rows;
	p.columns = 1;
	p.mat = workarray;
	XXIndex(&p);					/* create p = [0,1,2, ...., rows]         */
	det = XXDecompose(dest, &p);	/* this changes destination matrix itself. */

	/* if the determinant is zero there is no solution possible.
	 * so return this determinant so that the caller is informed about
	 * this fact.
	 */
	if( det == 0.0)
		return 0.0;

	/* use the offset in the workarray as work array for the permute operation */
	permwork = &workarray[p.rows];
	XXPermute(v, &p, permwork);				/* swap the vector v using the swap vector p */
	XXSubstitute(dest, v);
	return det;
}


/* this crout routine preserves the values of this.
 * and stores the  changed values of the crouted matrix
 * in CroutMat. On output the vector x contains the answer.
 * and y is preserved
 * workarray must be at least 2 * dest->rows long
 */
XXDouble XXCrout2 (XXMatrix *dest, XXMatrix *CroutMat,
			 XXMatrix *x, XXMatrix *y, XXDouble *workarray)
{
	/* preserve memory */
	XXMatrixMov(CroutMat, dest);
	XXMatrixMov(x, y);
	return XXCrout1(CroutMat, x, workarray);
}


/* the inverse function calculates the matrix inverse
 * and returns the determinant.
 * workarray size at least (3 * n * n + 2 * n)
 * with n is rows is columns from source (=equal to destination)
 */
XXDouble XXInverse (XXMatrix *mat_dest, XXMatrix *mat_source, XXDouble *workarray)
{
	/* wrapper matrices for the workarray */
	XXMatrix newMatrix;
	XXMatrix p;
	XXMatrix t;
	XXMatrix tempV;
	XXInteger i, offset, rows, columns;
	XXDouble det;

	rows = mat_source->rows;
	columns = mat_source->columns;

	/* map the workarray onto the the matrix objects */
	newMatrix.rows = rows;
	newMatrix.columns = columns;
	newMatrix.mat = workarray;
	offset = rows * columns;

	p.rows = rows;
	p.columns = 1;
	p.mat = &workarray[offset];
	offset += rows;

	t.rows = rows;
	t.columns = columns;
	t.mat = &workarray[offset];
	offset += rows * columns;

	tempV.rows = 1;
	tempV.columns = columns;
	tempV.mat = &workarray[offset];
	offset += columns;

	/* and do the calculations */
	XXIndex(&p);
	XXMatrixEye(&t);

	/* make a copy of the source, since we are going to chenge this */
	XXMatrixMov(&newMatrix, mat_source);
	det = XXDecompose(&newMatrix, &p);
	if( det == 0.0 )
	{
		/* there is no inverse of the matrix */
		return 0.0;
	}

	/* treat col vectors as row vectors. */
	for( i = 0; i < rows; i++)
	{
		XXMatrixGetRow(&tempV, &t, i);

		/* workarray here size of v */
		XXPermute( &tempV, &p, &workarray[offset]);
		XXSubstitute(&newMatrix, &tempV );

		XXMatrixSetRow(&t, &tempV, i);
	}

	/* transpose to col vectors to our final destination */
	XXMatrixTranspose(mat_dest, &t);

	return det;
}


/* take the determinant of the matrix source and put it in the scalar destination *
 * simply by reusing a lot of the inverse functionality 
 * NOTE: can be improved */ 
XXDouble XXMatrixDeterminant (XXMatrix *mat_source, XXDouble *workarray)
{
	/* wrapper matrices for the workarray */
	XXMatrix newMatrix;
	XXMatrix p;
	XXInteger rows, columns;

	rows = mat_source->rows;
	columns = mat_source->columns;

	/* map the workarray onto the the matrix objects */
	newMatrix.rows = rows;
	newMatrix.columns = columns;
	newMatrix.mat = workarray;

	p.rows = rows;
	p.columns = 1;
	p.mat = &workarray[rows * columns];

	/* and do the calculations */
	XXIndex(&p);

	/* make a copy of the source, since we are going to chenge this */
	XXMatrixMov(&newMatrix, mat_source);
	
	/* and return the determinant */
	return XXDecompose(&newMatrix, &p);
}


/* take the inverse of a matrix source to a matrix destination
 * workarray size at least (3 * n * n + 2 * n)
 * with n is rows is columns from source (=equal to destination)
 */
void XXMatrixInverse (XXMatrix *mat_dest, XXMatrix *mat_source, XXDouble *workarray)
{
	XXDouble det;
	det = XXInverse(mat_dest, mat_source, workarray);

	/* here you could do something with the det result */
}


/* divide matrix source1 with matrix source2 to matrix destination
 * workarray size at least (4 * n * n + 2 * n)
 * with n is rows is columns from source2
 */
void XXMatrixDiv (XXMatrix *mat_dest, XXMatrix *mat_source1, XXMatrix *mat_source2, XXDouble *workarray)
{
	XXMatrix workMatrix;
	XXInteger offset;
	XXDouble det;

	workMatrix.rows = mat_source2->rows;
	workMatrix.columns = mat_source2->columns;
	workMatrix.mat = workarray;
	offset = workMatrix.rows * workMatrix.columns;

	/* take the inverse and put it in our workmatrix */
	det = XXInverse(&workMatrix, mat_source2, &workarray[offset]);

	if( det == 0.0 )
		return;

	/* multiply source 1 with the workMatrix(=inverse(source1)) */
	XXMatrixMul(mat_dest, mat_source1, &workMatrix);
}


/* divide matrix source1 with scalar source2 to matrix destination 
*/
void XXMatrixScalarDiv (XXMatrix *mat_dest, XXMatrix *mat_source1, XXDouble s2)
{
	XXDouble *d, *s1;
	XXInteger i;

	d = mat_dest->mat;
	s1 = mat_source1->mat;
	i = mat_dest->rows * mat_dest->columns;

	while (i)
	{
		(*d) = (*s1) / s2;
		d++; s1++; i--;
	}
}


/* divide scalar source1 with matrix source2 to matrix destination
 * workarray size at least (3 * n * n + 2 * n)
 * with n is rows is columns from source2 (=equal to destination)
 */
void XXScalarMatrixDiv (XXMatrix *mat_dest, XXDouble s1, XXMatrix *mat_source2, XXDouble *workarray)
{
	XXDouble det;
	XXInteger i, size;
	det = XXInverse(mat_dest, mat_source2, workarray);

	/* here you could do something with the det result */
	if( det == 0.0 )
		return;

	/* and multiply by the scalar */
	size = mat_dest->rows * mat_dest->columns;
	for( i = 0; i < size; i++)
		mat_dest->mat[i] *= s1;
}

