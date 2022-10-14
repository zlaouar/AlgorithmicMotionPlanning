#include <iostream>

using namespace std;

int main(){
    
    for(int i=0; i<5; i++){
        cout << "i: " << i <<endl;
    }
    cout << "____" << endl;
    for(int i=0; i<5; ++i){
        cout << "i: " << i <<endl;
    }
    
    return 0;
}