#ifndef SIGNAL_MATCHER_H
#define SIGNAL_MATCHER_H

#define MATCH_ERROR_THRESHOLD 0
/* #define MATCH_ERROR_THRESHOLD 1 */

#include <iostream>
namespace uvdar {

  class SignalMatcher{
    public:
      SignalMatcher(std::vector<std::vector<bool>> i_sequences, int i_allowed_BER_per_seq_){

        allowed_BER_per_seq_ = i_allowed_BER_per_seq_;
        sequences_ = i_sequences;
        sequence_size_ = sequences_.at(0).size();
        for (auto &curr_seq : sequences_){
          auto curr_seq_copy = curr_seq;
          // append the original signal at the end and delete last Bit e.g. 0,1 -> 0,1,0 
          curr_seq.insert(curr_seq.end(),curr_seq_copy.begin(),curr_seq_copy.end()-1);
        }
      }

      SignalMatcher(std::vector<std::vector<bool>> i_sequences){
        //TODO sanitation
        sequences_ = i_sequences;
        sequence_size_ = sequences_.at(0).size();
        for (auto &curr_seq : sequences_){
          auto curr_seq_copy = curr_seq;
          // append the original signal at the end and delete last Bit e.g. 0,1 -> 0,1,0 
          curr_seq.insert(curr_seq.end(),curr_seq_copy.begin(),curr_seq_copy.end()-1);
        }
      }

      int matchSignal(std::vector<bool> i_signal){
        for (int s=0; s<(int)(sequences_.size()); s++){ // check sequences
          for (int i=0; i<sequence_size_; i++){ //slide along the duplicated sequence
            int match_errors = 0;
            for (int j=0; j<(int)(i_signal.size()); j++){ //iterate over signal
              if (sequences_.at(s).at(i+j) != i_signal.at(j)){
                match_errors++;
              }
              if (match_errors > MATCH_ERROR_THRESHOLD) {//TODO make settable
                break;
              }
            }
            if (match_errors <= MATCH_ERROR_THRESHOLD){
              return s;
            }
          }
        }
        return -1;

      }

      int matchSignalWithCrossCorr(std::vector<bool> i_signal){

        if (i_signal.size() == 0){
          return -1;
        }

        if (i_signal.size() < 3){
          return -3;
        }

        for (int s=0; s<(int)(sequences_.size()); s++){
          for (int i=0; i<(int)sequences_[s].size(); i++){
            int corr_val = 0;
            for (int j=0; j<(int)i_signal.size(); j++){
              if(sequences_[s][i+j] == i_signal[j]){
                corr_val++;
              }
            }
            if (corr_val == sequence_size_){
              return s;
            }
            int valid_bits = sequence_size_ - allowed_BER_per_seq_;
            if(corr_val >= valid_bits ){
              return s; 
            }
          }
        }
        return -1; 
      }


    private:

      
  /**
   * Atrributes
   */

  std::vector<std::vector<bool>> sequences_;
  int sequence_size_;
  int allowed_BER_per_seq_ = 0;

  };
}


#endif // SIGNAL_MATCHER_H
