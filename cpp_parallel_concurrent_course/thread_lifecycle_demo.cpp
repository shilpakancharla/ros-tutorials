/**
 * Two threads cooking soupe
 */
# include <thread>
# include <chrono>

void chef_gojo() {
  printf("Gojo started & waiting for something to thaw...\n");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  printf("Gojo is done cutting.\n");
}
         
int main() {
  printf("Nanami requests Gojo's help.\n");
  std::thread gojo(chef_gojo);
  
  printf("Nanami continues cooking soup.\n");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  printf("Nanami patiently waits for Gojo to finish and join...\n");
  gojo.join();
  
  printf("Nanami and Gojo are both done!\n");
}
