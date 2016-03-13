#include "test.h"

int main (int argc, char ** argv) {
    test::TestEquality();
    test::TestMatrixDeterminant();
    test::TestInverse();
    test::TestViewProjectionInverse();
    test::TestMatrixMultiplication1();
    test::TestMatrixMultiplication2();
    test::TestQuaternions();
    test::TestQuaternions2();
    test::TestQuaternions3();
    test::TestClipping();
    test::TestFrustum();

    return 0;
}
