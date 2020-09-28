#include <iostream>

using namespace std;
namespace robot {
	void process(void) {
		cout<<"Processing by Robot"<<endl;
	}
}

namespace machine {
	void process(void) {
		cout<<"Processing by Machine"<<endl;
	}
}

int main(void) {
	robot::process();
	machine::process();
	return 0;
}
