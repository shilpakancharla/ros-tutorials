/**
 * Two shoppers adding items to a shared notepad
 */
# include <thread>
# include <mutex>

unsigned int garlic_count = 0;
std::mutex pencil;

void shopper() {
  pencil.lock();
  for (int i = 0; i < 1000000; i++) {
    garlic_count++;
  }
  pencil.unlock();
}

int main() {
  std::thread nanami(shopper);
  std::thread gojo(shopper);
  nanami.join();
  gojo.join();
  printf("We should buy %u garlic.\n", garlic_count);
}
