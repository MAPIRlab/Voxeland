#pragma once

class PipelineStep {
    public:
        virtual ~PipelineStep() = default;
        virtual void execute() = 0;
};
