#include "disambiguation/pipeline/pipeline_steps.hpp"

void AbstractPipelineStep::set_next(PipelineStep* next_step){
    this->next = next_step;
}

void AbstractPipelineStep::execute_next(){
    if (next != nullptr){
        next->execute();
    }
}