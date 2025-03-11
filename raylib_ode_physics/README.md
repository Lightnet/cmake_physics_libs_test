# Information:

# Required:
 * VS2022
 * CMake

# Notes:
 * https://ode.org/wiki/index.php/Manual

```c
//rotate data
Matrix rot_matrix = {
    rot[0], rot[1], rot[2], 0,
    rot[4], rot[5], rot[6], 0,
    rot[8], rot[9], rot[10], 0,
    0, 0, 0, 1 //this need 1 not zero else strange error
};
```

```c
//not correct rotate set back Grok Beta 3 mistake somewhere.
// Matrix rot_matrix = {
//     rot[0], rot[4], rot[8], 0,
//     rot[1], rot[5], rot[9], 0,
//     rot[2], rot[6], rot[10], 0,
//     0, 0, 0, 1
// };
```

```
void convertMatrix(D3DXMATRIX &t, const dReal *bMat)
{
    t._11 = bMat[0];
    t._12 = bMat[1];
    t._13 = bMat[2];
    t._14 = 0;
    t._21 = bMat[4];
    t._22 = bMat[5];
    t._23 = bMat[6];
    t._24 = 0;
    t._31 = bMat[8];
    t._32 = bMat[9];
    t._33 = bMat[10];
    t._34 = 0;
}
```
 * https://ode.org/wiki/index.php/Manual
 * example refs.