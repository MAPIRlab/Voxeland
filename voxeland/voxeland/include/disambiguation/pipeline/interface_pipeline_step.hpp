#pragma once

class PipelineStep {
    public:
        virtual ~PipelineStep() = default;
        virtual bool execute() = 0;
};
