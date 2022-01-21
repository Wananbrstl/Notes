#include<iostream>
#include"gtest/gtest.h"

using namespace std;

template<class T>
T getAdd(T a, T b)
{
    return (a + b);
}

TEST(getAdd, test)
{
    ASSERT_EQ(getAdd(1,2),3);
    ASSERT_EQ(getAdd(0,1),1);
    ASSERT_EQ(getAdd(1.0,3.5),4.5);
    ASSERT_EQ(getAdd(3.0,4.0),7.0);
    ASSERT_EQ(getAdd(0,0),0);
}
