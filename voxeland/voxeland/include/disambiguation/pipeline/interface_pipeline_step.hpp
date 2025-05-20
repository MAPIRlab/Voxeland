
class PipelineStep{
    public:
        virtual void set_next(PipelineStep* next_step) = 0;
        virtual void execute() = 0;
    protected:
        PipelineStep* next;
};