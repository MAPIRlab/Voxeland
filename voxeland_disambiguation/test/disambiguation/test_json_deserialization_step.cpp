#include <gtest/gtest.h>
#include "disambiguation/pipeline/pipeline_steps.hpp"

TEST(JsonDeserializationStep, test_execute_non_existing_map_throws_error) {
    JsonDeserializationStep step("test_map.json", "test_appearances.json");
    
    bool result = step.execute();

    ASSERT_FALSE(result);
}