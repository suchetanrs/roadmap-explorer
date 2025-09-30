#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

#include "roadmap_explorer/util/GeneralUtils.hpp"

class GeneralUtilsTest : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        // Set up common test data
        int_vector = {1, 2, 3, 4, 5};
        string_vector = {"apple", "banana", "cherry", "date"};
        double_vector = {1.1, 2.2, 3.3, 4.4, 5.5};
        char_vector = {'a', 'b', 'c', 'd'};
        empty_vector = {};
    }
    
    std::vector<int> int_vector;
    std::vector<std::string> string_vector;
    std::vector<double> double_vector;
    std::vector<char> char_vector;
    std::vector<int> empty_vector;
};

// Test vectorContains with int vector
TEST_F(GeneralUtilsTest, VectorContainsIntFound) 
{
    EXPECT_TRUE(vectorContains(int_vector, 3));
    EXPECT_TRUE(vectorContains(int_vector, 1));
    EXPECT_TRUE(vectorContains(int_vector, 5));
}

TEST_F(GeneralUtilsTest, VectorContainsIntNotFound) 
{
    EXPECT_FALSE(vectorContains(int_vector, 0));
    EXPECT_FALSE(vectorContains(int_vector, 6));
    EXPECT_FALSE(vectorContains(int_vector, -1));
    EXPECT_FALSE(vectorContains(int_vector, 100));
}

// Test vectorContains with string vector
TEST_F(GeneralUtilsTest, VectorContainsStringFound) 
{
    EXPECT_TRUE(vectorContains(string_vector, std::string("apple")));
    EXPECT_TRUE(vectorContains(string_vector, std::string("banana")));
    EXPECT_TRUE(vectorContains(string_vector, std::string("cherry")));
    EXPECT_TRUE(vectorContains(string_vector, std::string("date")));
}

TEST_F(GeneralUtilsTest, VectorContainsStringNotFound) 
{
    EXPECT_FALSE(vectorContains(string_vector, std::string("grape")));
    EXPECT_FALSE(vectorContains(string_vector, std::string("")));
    EXPECT_FALSE(vectorContains(string_vector, std::string("APPLE")));  // Case sensitive
    EXPECT_FALSE(vectorContains(string_vector, std::string("apple ")));  // Space matters
}

// Test vectorContains with double vector
TEST_F(GeneralUtilsTest, VectorContainsDoubleFound) 
{
    EXPECT_TRUE(vectorContains(double_vector, 1.1));
    EXPECT_TRUE(vectorContains(double_vector, 3.3));
    EXPECT_TRUE(vectorContains(double_vector, 5.5));
}

TEST_F(GeneralUtilsTest, VectorContainsDoubleNotFound) 
{
    EXPECT_FALSE(vectorContains(double_vector, 1.0));
    EXPECT_FALSE(vectorContains(double_vector, 1.11));  // Precision matters
    EXPECT_FALSE(vectorContains(double_vector, 6.6));
    EXPECT_FALSE(vectorContains(double_vector, 0.0));
}

// Test vectorContains with char vector
TEST_F(GeneralUtilsTest, VectorContainsCharFound) 
{
    EXPECT_TRUE(vectorContains(char_vector, 'a'));
    EXPECT_TRUE(vectorContains(char_vector, 'b'));
    EXPECT_TRUE(vectorContains(char_vector, 'c'));
    EXPECT_TRUE(vectorContains(char_vector, 'd'));
}

TEST_F(GeneralUtilsTest, VectorContainsCharNotFound) 
{
    EXPECT_FALSE(vectorContains(char_vector, 'e'));
    EXPECT_FALSE(vectorContains(char_vector, 'A'));  // Case sensitive
    EXPECT_FALSE(vectorContains(char_vector, '1'));
    EXPECT_FALSE(vectorContains(char_vector, ' '));
}

// Test vectorContains with empty vector
TEST_F(GeneralUtilsTest, VectorContainsEmptyVector) 
{
    EXPECT_FALSE(vectorContains(empty_vector, 1));
    EXPECT_FALSE(vectorContains(empty_vector, 0));
    EXPECT_FALSE(vectorContains(empty_vector, -1));
}

// Test vectorContains with single element vector
TEST_F(GeneralUtilsTest, VectorContainsSingleElement) 
{
    std::vector<int> single_element = {42};
    
    EXPECT_TRUE(vectorContains(single_element, 42));
    EXPECT_FALSE(vectorContains(single_element, 41));
    EXPECT_FALSE(vectorContains(single_element, 43));
}

// Test vectorContains with duplicate elements
TEST_F(GeneralUtilsTest, VectorContainsDuplicateElements) 
{
    std::vector<int> duplicate_vector = {1, 2, 2, 3, 3, 3};
    
    EXPECT_TRUE(vectorContains(duplicate_vector, 1));
    EXPECT_TRUE(vectorContains(duplicate_vector, 2));
    EXPECT_TRUE(vectorContains(duplicate_vector, 3));
    EXPECT_FALSE(vectorContains(duplicate_vector, 4));
}

// Test vectorContains with custom struct
struct TestStruct 
{
    int id;
    std::string name;
    
    bool operator==(const TestStruct& other) const 
    {
        return id == other.id && name == other.name;
    }
};

TEST_F(GeneralUtilsTest, VectorContainsCustomStruct) 
{
    std::vector<TestStruct> struct_vector = {
        {1, "first"},
        {2, "second"},
        {3, "third"}
    };
    
    TestStruct search_found = {2, "second"};
    TestStruct search_not_found1 = {2, "Second"};  // Case sensitive
    TestStruct search_not_found2 = {4, "fourth"};
    
    EXPECT_TRUE(vectorContains(struct_vector, search_found));
    EXPECT_FALSE(vectorContains(struct_vector, search_not_found1));
    EXPECT_FALSE(vectorContains(struct_vector, search_not_found2));
}

// Test vectorContains with large vector (performance test)
TEST_F(GeneralUtilsTest, VectorContainsLargeVector) 
{
    std::vector<int> large_vector;
    for (int i = 0; i < 10000; ++i) {
        large_vector.push_back(i);
    }
    
    EXPECT_TRUE(vectorContains(large_vector, 5000));
    EXPECT_TRUE(vectorContains(large_vector, 0));
    EXPECT_TRUE(vectorContains(large_vector, 9999));
    EXPECT_FALSE(vectorContains(large_vector, 10000));
    EXPECT_FALSE(vectorContains(large_vector, -1));
}

// Test vectorContains with negative numbers
TEST_F(GeneralUtilsTest, VectorContainsNegativeNumbers) 
{
    std::vector<int> negative_vector = {-5, -3, -1, 0, 1, 3, 5};
    
    EXPECT_TRUE(vectorContains(negative_vector, -5));
    EXPECT_TRUE(vectorContains(negative_vector, -1));
    EXPECT_TRUE(vectorContains(negative_vector, 0));
    EXPECT_TRUE(vectorContains(negative_vector, 5));
    EXPECT_FALSE(vectorContains(negative_vector, -2));
    EXPECT_FALSE(vectorContains(negative_vector, 2));
}

// Test vectorContains with pointer vector
TEST_F(GeneralUtilsTest, VectorContainsPointers) 
{
    int a = 10, b = 20, c = 30;
    std::vector<int*> pointer_vector = {&a, &b, &c};
    
    EXPECT_TRUE(vectorContains(pointer_vector, &a));
    EXPECT_TRUE(vectorContains(pointer_vector, &b));
    EXPECT_TRUE(vectorContains(pointer_vector, &c));
    
    int d = 40;
    EXPECT_FALSE(vectorContains(pointer_vector, &d));
}

// Test vectorContains with const vector
TEST_F(GeneralUtilsTest, VectorContainsConstVector) 
{
    const std::vector<int> const_vector = {10, 20, 30, 40, 50};
    
    EXPECT_TRUE(vectorContains(const_vector, 30));
    EXPECT_FALSE(vectorContains(const_vector, 35));
}

// Test edge cases with special double values
TEST_F(GeneralUtilsTest, VectorContainsSpecialDoubles) 
{
    std::vector<double> special_doubles = {
        0.0, -0.0, 
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max()
    };
    
    EXPECT_TRUE(vectorContains(special_doubles, 0.0));
    EXPECT_TRUE(vectorContains(special_doubles, std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(vectorContains(special_doubles, -std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(vectorContains(special_doubles, std::numeric_limits<double>::min()));
    EXPECT_TRUE(vectorContains(special_doubles, std::numeric_limits<double>::max()));
    
    EXPECT_FALSE(vectorContains(special_doubles, 1.0));
}

// Test with NaN (tricky case)
TEST_F(GeneralUtilsTest, VectorContainsNaN) 
{
    std::vector<double> nan_vector = {
        1.0, 2.0, std::numeric_limits<double>::quiet_NaN(), 3.0
    };
    
    // NaN should not equal itself, so this should return false
    EXPECT_FALSE(vectorContains(nan_vector, std::numeric_limits<double>::quiet_NaN()));
    
    // But regular numbers should still work
    EXPECT_TRUE(vectorContains(nan_vector, 1.0));
    EXPECT_TRUE(vectorContains(nan_vector, 2.0));
    EXPECT_TRUE(vectorContains(nan_vector, 3.0));
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}



