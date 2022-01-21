/**
 * @file test.cpp
 * @brief This is just for test the data structure
 * @author wananbrstl
 * @version 1.0
 * @date 2022-01-01
 */
#include<limits.h>
#include<iostream>
#include<vector>
#include<typeinfo>

using namespace std;

int main(int argc, char **argv)
{
    // 一些数据结构的占用空间
    int test_int = 3;
    char test_char = 1;
    long test_long = 1;
    long long test_long_long = 1;
    float test_float = 3.123123;
    double test_double = 3.121231231;
    // 打印信息
    printf("[INT] SIZE = %ld\n", sizeof(test_int));
    printf("[CHAR] SIZE = %ld\n", sizeof(test_char));
    printf("[LONG] SIZE = %ld\n", sizeof(test_long));
    printf("[LONG LONG] SIZE = %ld\n", sizeof(test_long_long));
    printf("[DOUBLE] SIZE = %ld\n", sizeof(test_double));
    printf("[FLOAT] SIZE = %ld\n", sizeof(test_float));

    printf("Type\tmax\tmin\t\n");
    printf("int\t %d \t%d\n",INT_MAX, INT_MIN);
    
    // 算术算法
    // double / float
    cout << "double / flaot " << typeid(test_double/test_float).name() << endl;
    cout << "double / int " << typeid(test_double/test_int).name() << endl;
    cout << "float / int " << typeid(test_float/test_int).name() << endl;

    cout << "int / char " << typeid(test_int/test_char).name() << endl;
    cout << "doule / char " << typeid(test_double/test_char).name() << endl;
    cout << "float / char " << typeid(test_float/test_char).name() << endl;

    unsigned long long a = 1;

    cout << "doule / ulonglong " << typeid(test_double/a).name() << endl;
    
    int N = 100;
    vector<double> x(N);
    cout << "x.size = " << x.size() << endl;
    for(auto i: x)
        cout << i << endl;
    
    return 0;

}
