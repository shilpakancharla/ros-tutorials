/**
 * Two threads chopping vegetables
 */
# include <thread>
# include <chrono>

void kitchen_cleaner() {
  while (true) {
    printf("Gojo cleaned the kitchen.\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main() {
  std::thread gojo(kitchen_cleaner);
  gojo.detach(); //Runs detached, no need for join function
  for (int = 0; i < 3; i++) {
    printf("Nanami is cooking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
  }  
  printf("Nanami is done!\n");
}
