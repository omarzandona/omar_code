#include "hungarian_method.h"
#include <assert.h>

//For each row of the cost matrix, find the smallest element and subtract
//it from every element in its row.  When finished, Go to Step 2.
static void step_one(int & step, unsigned int * const & C, const int nrow, const int ncol)
 {
  for (int r = 0; r < nrow; ++r)
  {
    int offset = r*ncol;

    unsigned int min_in_row = C[offset+0];
    for (int c = 1; c < ncol; ++c)
      if (C[offset+c] < min_in_row)
        min_in_row = C[offset+c];

    for (int c = 0; c < ncol; ++c)
    {
#ifdef _DEBUG
      assert(C[offset+c]>=min_in_row);
#endif
      C[offset+c] -= min_in_row;
    }
  }
  step = 2;
}

//Find a zero (Z) in the resulting matrix.  If there is no starred 
//zero in its row or column, star Z. Repeat for each element in the 
//matrix. Go to Step 3.
static void step_two(int & step, 
                     const unsigned int * const & C, 
                     int * const & RowCover, int * const & ColCover, 
                     int * const & M, 
                     const int nrow, const int ncol)
{
  for (int r = 0; r < nrow; ++r)
  {
    int offset = r*ncol;
    for (int c = 0; c < ncol; ++c)
    {
      if (C[offset+c] == 0 && RowCover[r] == 0 && ColCover[c] == 0)
      {
        M[offset+c] = 1;
        RowCover[r] = 1;
        ColCover[c] = 1;
      }
    }
  }

  memset(RowCover, 0, nrow*sizeof(int));
  memset(ColCover, 0, ncol*sizeof(int));

  step = 3;
}

//Cover each column containing a starred zero.  If K columns are covered, 
//the starred zeros describe a complete set of unique assignments.  In this 
//case, Go to DONE, otherwise, Go to Step 4.
static void step_three(int & step, 
                       const int * const & M, 
                       const int nrow, const int ncol, 
                       int * const & ColCover)
{
  for (int r = 0; r < nrow; ++r)
  {
    int offset = r*ncol;
    for (int c = 0; c < ncol; ++c)
    {
      if (M[offset+c] == 1)
        ColCover[c] = 1;
    }
  }

  int colcount = 0;
  for (int c = 0; c < ncol; ++c)
  {
    if (ColCover[c] == 1)
      colcount++;
  }

  if (colcount >= ncol || colcount >=nrow)
    step = 7;
  else
    step = 4;
}

//methods to support step 4
static void find_a_zero(int & row, int & col, 
                        const unsigned int * const & C, 
                        const int * const & RowCover, const int * const & ColCover, 
                        const int nrow, const int ncol)
{
  int r, c;
  bool done;
  
  row = col = -1;
  done = false;
  r = 0;
  while (r<nrow && !done)
  {
    c = 0;
    int offset = r*ncol;
    while (c<ncol && !done)
    {
      if (C[offset+c] == 0 && RowCover[r] == 0 && ColCover[c] == 0)
      {
        row = r;
        col = c;
        done = true;
      }
      ++c;
    }
    ++r;
  }
}

static int find_star_in_row(const int row, const int * const & M, const int nrow, const int ncol)
{
  int col = -1;
  int offset = row*ncol;
  for (int c = 0; c < ncol; ++c)
    if (M[offset+c] == 1)
      col = c;
  return col;
}

//Find a noncovered zero and prime it.  If there is no starred zero 
//in the row containing this primed zero, Go to Step 5.  Otherwise, 
//cover this row and uncover the column containing the starred zero. 
//Continue in this manner until there are no uncovered zeros left. 
//Save the smallest uncovered value and Go to Step 6.
static void step_four(int & step, unsigned int * const & C,  
                      int * const & RowCover, int * const & ColCover, 
                      int * const & M, 
                      const int nrow, const int ncol,
                      int & path_row_0, int & path_col_0)
{
  int row = -1;
  int col = -1;
  path_row_0 = -1;
  path_col_0 = -1;

  bool done;

  done = false;
  while (!done)
  {
    find_a_zero(row, col, C, RowCover, ColCover, nrow, ncol);
    if (row == -1)
    {
      done = true;
      step = 6;
    }
    else
    {
      M[row*ncol+col] = 2;
      int tmp = find_star_in_row(row, M, nrow, ncol);
      if (tmp > -1)
      {
        col = tmp;
        RowCover[row] = 1;
        ColCover[col] = 0;
      }
      else
      {
        done = true;
        step = 5;
        path_row_0 = row;
        path_col_0 = col;
      }
    }
  }
}

// methods to support step 5
static int find_star_in_col(int c, int * const & M, const int nrow, const int ncol)
{
  assert(c < ncol);
  assert(c >= 0);

  int r = -1;
  for (int i = 0; i < nrow; ++i)
    if (M[i*ncol+c] == 1)
      r = i;
  return r;
}

static int find_prime_in_row(int r, int * const & M, const int nrow, const int ncol)
{
  assert(r < nrow);
  assert(r >= 0);

  int c = -1;
  for (int j = 0; j < ncol; j++)
    if (M[r*ncol+j] == 2)
      c = j;
  return c;
}

static void augment_path(int * const & M, const int nrow, const int ncol, 
                         const int * const & path, const int path_count)
{
  assert(path_count < nrow*ncol+1);
  assert(path_count >= 0);

  for (int p = 0; p < path_count; ++p)
  {
    int offset = p*2;
    int r = path[offset+0];
    int c = path[offset+1];

    assert(r < nrow);
    assert(r >= 0);
    assert(c < ncol);
    assert(c >= 0);

    int idx = r*ncol + c;

    if (M[idx] == 1)
      M[idx] = 0;
    else
      M[idx] = 1;
  }
}

static void clear_covers(int * const & RowCover, int * const & ColCover, 
                         const int nrow, const int ncol)
{
  for (int r = 0; r < nrow; ++r)
    RowCover[r] = 0;
  for (int c = 0; c < ncol; ++c)
    ColCover[c] = 0;
}

static void erase_primes(int * const & M, const int nrow, const int ncol)
{
  for (int r = 0; r < nrow; ++r)
  {
    int offset = r*ncol;
    for (int c = 0; c < ncol; ++c)
      if (M[offset+c] == 2)
        M[offset+c] = 0;
  }
}


//Construct a series of alternating primed and starred zeros as follows.  
//Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote 
//the starred zero in the column of Z0 (if any). Let Z2 denote the primed zero 
//in the row of Z1 (there will always be one).  Continue until the series 
//terminates at a primed zero that has no starred zero in its column.  
//Unstar each starred zero of the series, star each primed zero of the series, 
//erase all primes and uncover every line in the matrix.  Return to Step 3.
static void step_five(int & step, int * const & M, int * const & RowCover, int * const & ColCover, 
                      const int nrow, const int ncol, int * const & path, int & path_count, 
                      const int path_row_0, const int path_col_0)
{
  assert(path_row_0 >= 0);
  assert(path_row_0 < nrow);
  assert(path_col_0 >= 0);
  assert(path_col_0 < ncol);

  bool done;

  path_count = 1;
  path[0] = path_row_0;
  path[1] = path_col_0;

  done = false;
  while (!done)
  {
    assert(path_count < nrow*ncol+1);
    assert(path_count >= 1);

    int col = path[(path_count - 1)*2+1];
    int r = find_star_in_col(col, M, nrow, ncol);
    if (r > -1)
    {
      path_count++;
      int offset = (path_count - 1)*2;
      path[offset+0] = r;
      path[offset+1] = col;
    }
    else
      done = true;

    if (!done)
    {
      int row = path[(path_count - 1)*2+0];
      int c = find_prime_in_row(row, M, nrow, ncol);

      path_count++;
      int offset = (path_count - 1)*2;
      path[offset+0] = row;
      path[offset+1] = c;
    }
  }

  augment_path(M, nrow, ncol, path, path_count);
  clear_covers(RowCover, ColCover, nrow, ncol);
  erase_primes(M, nrow, ncol);

  step = 3;
}

//methods to support step 6
static void find_smallest(unsigned int & minval, unsigned int * const & C, 
                          int * const & RowCover, int * const & ColCover, 
                          const int nrow, const int ncol)
{
  minval = INT_MAX;
  for (int r = 0; r < nrow; ++r)
    for (int c = 0; c < ncol; ++c)
      if (RowCover[r] == 0 && ColCover[c] == 0)
        if (minval > C[r*ncol+c])
          minval = C[r*ncol+c];
}

//Add the value found in Step 4 to every element of each covered row, and subtract 
//it from every element of each uncovered column.  Return to Step 4 without 
//altering any stars, primes, or covered lines.
static void step_six(int & step, unsigned int * const & C, 
                     int * const & RowCover, int * const & ColCover, 
                     const int nrow, const int ncol)
{
  unsigned int minval;
  find_smallest(minval, C, RowCover, ColCover, nrow, ncol);

  assert(minval >= 0);
  for (int r = 0; r < nrow; ++r)
  {
    int offset = r*ncol;
    for (int c = 0; c < ncol; ++c)
    {
      if (RowCover[r] == 1)
      {
        if ((UINT_MAX-C[offset+c]) <= minval)
          C[offset+c] = UINT_MAX;
        else
          C[offset+c] += minval;
      }
      if (ColCover[c] == 0)
      {
#ifdef _DEBUG
        assert(C[offset+c]>=minval);
#endif
        C[offset+c] -= minval;
      }
    }
  }

  step = 4;
}

int* matching(const unsigned int * const & i_C, const int nrow, const int ncol, const int max_cost, int * p_result)
{
  assert(nrow>0 && ncol>0);
  assert(nrow == ncol);

  static int last_sz = -1;
  static unsigned int* C = NULL;

  int sz = nrow*ncol*sizeof(unsigned int);
  if (last_sz != sz)
  {
    last_sz = sz;
    if (C != NULL)
      free(C);
    C = (unsigned int*) malloc(nrow*ncol*sizeof(unsigned int));
  }

  memcpy(C, i_C, sz);

#ifdef _DEBUG
  for (int r=0; r<nrow; ++r)
  {
    int offset = r*ncol;
    int num_elem_to_max = 0;
    for (int c=0; c<ncol; ++c)
    {
      if (C[offset+c] == max_cost)
        num_elem_to_max++;
    }
    assert(num_elem_to_max != ncol);
  }

  for (int c=0; c<ncol; ++c)
  {
    int num_elem_to_max = 0;
    for (int r=0; r<nrow; ++r)
    {
      int offset = r*ncol;
      if (C[offset+c] == max_cost)
        num_elem_to_max++;
    }
    assert(num_elem_to_max != nrow);
  }
#endif

  int step = 1;
  bool done = false;

  int sze = nrow*sizeof(int);
  int *RowCover = (int*) malloc(sze);

  sze = ncol*sizeof(int);
  int *ColCover = (int*) malloc(sze);

  sze = 2*(nrow+ncol+1)*sizeof(int);
  int *path = (int*) malloc(sze);

  int path_count = 0;
  int path_row_0 = -1;
  int path_col_0 = -1;

  int * M;
  if (p_result == NULL)
    M = (int *) malloc(nrow*ncol*sizeof(int));
  else
    M = p_result;

  // reset mask and covers
  memset(M, 0, nrow*ncol*sizeof(int));
  memset(RowCover, 0, nrow*sizeof(int));
  memset(ColCover, 0, ncol*sizeof(int));

#ifdef _DEBUG
  unsigned long num_iter = 0;
#endif

  // run algorithm
  while (!done)
  {
    switch (step)
    {
    case 1:
      step_one(step, C, nrow, ncol);
      break;
    case 2:
      step_two(step, C, RowCover, ColCover, M, nrow, ncol);
      break;
    case 3:
      step_three(step, M, nrow, ncol, ColCover);
      break;
    case 4:
      step_four(step, C, RowCover, ColCover, M, nrow, ncol, path_row_0, path_col_0);
      break;
    case 5:
      step_five(step, M, RowCover, ColCover, nrow, ncol, path, path_count, path_row_0, path_col_0);
      break;
    case 6:
      step_six(step, C, RowCover, ColCover, nrow, ncol);
      break;
    case 7:
      done = true;
      break;
    }
#ifdef _DEBUG
    ++num_iter;
#endif
  }

#ifdef _DEBUG
  static unsigned long max_num_iter = 9;
  if (num_iter > max_num_iter)
  {
    printf("Max. num. iterations = %d\n", num_iter);
    max_num_iter = num_iter;
  }
#endif

  free(RowCover);
  free(ColCover);
  free(path);

  return M;
}
