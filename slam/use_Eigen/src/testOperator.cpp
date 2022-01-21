#include<iostream>

using namespace std;

// TODO &就是可以改变原来的意思
class A{
    public:
        A(){}

        A operator<<(A b)
        {
            this->m_num += b.getNum();
            return *this;
        }
        A operator<<(double b)
        {
            this->m_num += b;
            return *this;
        }

        void  setNum(double num) {this->m_num = num;}
        double getNum() const  {return m_num;}

    private:
        double m_num;
};

// TODO  只能重载参数， 不能重载返回值
template<class T_>
pair<T_,T_> foo(T_ a, T_ b)
{
    T_ temp;
    temp = a;
    a = b;
    b = temp;
    pair<T_,T_> p;
    p.first = a;
    p.second = b;
    return p;
}

// template<class T>
// void foo(T a, T b)
// {
//     T temp;
//     temp = a;
//     a = b;
//     b = temp;
// }

int main(int argc, char **argv)
{
    A a;
    a.setNum(1.0);
    cout << (a <<1<<1<<1).getNum() << endl;
    A b = a << 1 << 1<< 1;
    cout << "a  = " << a.getNum() <<endl;
    cout << "b  = " << b.getNum() <<endl;

    double num1 = 1.0, num2 = 2.0;
    cout <<"a = " << num1 << "  b = " << num2 << endl;
    pair<double, double> p;
    // p = foo(a_,b_);
    p = foo(num1,num2);
    cout << "swap_(): a = " <<p.second << "  b = " << p.second << endl; 


    return 0;
}
