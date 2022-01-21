#include <iostream>
using namespace std;

template<typename T, int a, int b> 
struct test
{
public:
    test(): x(a), y(b)
    {
    }
    void disp() const 
    {
        cout << "x = " << x << endl << "y = " << y << endl;
    }
private:
    T x;
    T y;
};
int main()
{
    test<int,1,2> a;
    test<double, 1, 2> b;
    a.disp();
    b.disp();
    return 0;
}
