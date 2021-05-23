/**
 * Two threads chopping vegetables
 */
# include <thread>
# include <chrono>

bool chopping = true;

void vegetable_chopper(const char* name) {
  unsigned int vegetable_count = 0;
  while (chopping) {
    vegetable_count++;
  }
  printf("%s chopped %u vegetables.\n", name, vegetable_count);
}

int main() {
  std::thread gojo(vegetable_chopper, "Gojo");
  std::thread nanami(vegetable_chopper, "Nanami");
  
  printf("Gojo and Nanami are chopping vegetables...\n");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  chopping = false;
  gojo.join();
  nanami.join();
}
