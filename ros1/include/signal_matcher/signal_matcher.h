#ifndef SIGNAL_MATCHER_H
#define SIGNAL_MATCHER_H

#define MATCH_ERROR_THRESHOLD 0
/* #define MATCH_ERROR_THRESHOLD 1 */

#include <optional>

namespace uvdar {

  class SignalMatcher{
    public:
      SignalMatcher(std::vector<std::vector<bool>> i_sequences, const int i_allowed_BER_per_seq, const std::optional<bool>& i_using_ami = std::nullopt){

        if(i_using_ami) using_ami = i_using_ami.value();
        else using_ami = false; 

        allowed_BER_per_seq_ = i_allowed_BER_per_seq;
        sequences_ = i_sequences;
        sequence_size_ = sequences_.at(0).size();
        for (auto &curr_seq : sequences_){
          auto curr_seq_copy = curr_seq;
          // append the original signal at the end and delete last Bit e.g. 0,1 -> 0,1,0 
          curr_seq.insert(curr_seq.end(),curr_seq_copy.begin(),curr_seq_copy.end()-1);
        }
      }

      int matchSignal(std::vector<bool> i_signal){

        if(using_ami){
          int valid_size = this->check_seq_size(i_signal);
          if(valid_size != 1){
            return valid_size;
          }
        }

        for (int s=0; s<(int)(sequences_.size()); s++){ // check sequences
          for (int i=0; i<sequence_size_; i++){ //slide along the duplicated sequence
            int match_errors = 0;
            for (int j=0; j<(int)(i_signal.size()); j++){ //iterate over signal
              if (sequences_.at(s).at(i+j) != i_signal.at(j)){
                match_errors++;
              }
              if (match_errors > allowed_BER_per_seq_) {
                break;
              }
            }
            if (match_errors <= allowed_BER_per_seq_){
              return s;
            }
          }
        }
        return -1;

      }

    private:

      int check_seq_size(const std::vector<bool> i_signal){

        if ((int)i_signal.size() == 0){
          return -1;
        }

        if ((int)i_signal.size() < sequence_size_){
          return -3;
        }
        return 1;
      }
      
  /**
   * Atrributes
   */

  std::vector<std::vector<bool>> sequences_;
  int sequence_size_;
  int allowed_BER_per_seq_ = 0;
  bool using_ami = true;

  };
}


#endif // SIGNAL_MATCHER_H
