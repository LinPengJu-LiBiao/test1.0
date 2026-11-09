#include <iostream>

double add(double a, double b) { return a + b; }
double sub(double a, double b) { return a - b; }
double mul(double a, double b) { return a * b; }
double div(double a, double b) { return a / b; }

void printMenu() {
    std::cout << "===== 简易计算器 =====\n"
              << "1) 加法   2) 减法   3) 乘法   4) 除法   0) 退出\n";
}

int main() {
    printMenu();

    while (true) {
        int op = 0;
        std::cout << "请选择操作: ";
        if (!(std::cin >> op)) break;
        if (op == 0) break;
        if (op < 1 || op > 4) {
            std::cout << "没有这个操作，请重新选择。\n";
            continue;
        }

        double a = 0.0, b = 0.0;
        std::cout << "请输入两个数: ";
        if (!(std::cin >> a >> b)) break;

        switch (op) {
            case 1:
                std::cout << a << " + " << b << " = " << add(a, b) << "\n";
                break;
            case 2:
                std::cout << a << " - " << b << " = " << sub(a, b) << "\n";
                break;
            case 3:
                std::cout << a << " * " << b << " = " << mul(a, b) << "\n";
                break;
            case 4:
                if (b == 0.0) {
                    std::cout << "除数不能为 0。\n";
                } else {
                    std::cout << a << " / " << b << " = " << div(a, b) << "\n";
                }
                break;
        }
    }

    std::cout << "已退出。\n";
    return 0;
}



/******************* */
