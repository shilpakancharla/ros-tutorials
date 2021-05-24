/**
 * Two shoppers adding items to a shared notepad
 */
# include <thread>
# include <mutex>
# include <chrono>

unsigned int items_on_notepad = 0;
std::mutex pencil;

void shopper() {
  int items_to_add = 0;
  while (items_on_notepad <= 20) {
    if (items_to_add && pencil.try_lock()) { // Add item(s) to shared items_on_notepad
      pencil.lock();
      items_on_notepad += items_to_add;
      printf("%s added %u item(s) to notepad.\n", name, items_on_add);
      items_to_add = 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Time spent writing
      pencil.unlock()
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Time spent searching
      items_to_add++;
      printf("%s found something else to buy.\n", name);
    }
  }
}

int main() {
  auto start_time = std::chrono::steady_clock::now();
  std::thread nanami(shopper, "Nanami");
  std::thread gojo(shopper, "Gojo");
  nanami.join();
  gojo.join();
  auto elapsed_time = std::chrono::duration_cost<std::chrono::milliseconds>(std::chrono::steady_clock);
  printf("We should buy %u garlic.\n", garlic_count);
}
