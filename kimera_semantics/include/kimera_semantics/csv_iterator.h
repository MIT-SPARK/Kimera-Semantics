/**
 * @file   csv_iterator.h
 * @brief  Parser for CSV files
 * @author Antoni Rosinol
 */

#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace kimera {

/* Usage:
 * std::ifstream file("plop.csv");
 *  for(CSVIterator loop(file); loop != CSVIterator(); ++loop)
 *  {
 *      std::cout << "4th Element(" << (*loop)[3] << ")\n";
 *  }
 */
class CSVIterator {
 private:
  class CSVRow {
   public:
    std::string const& operator[](std::size_t index) const;
    std::size_t size() const;
    void readNextRow(std::istream& str);

   private:
    std::vector<std::string> m_data;
  };

  friend std::istream& operator>>(std::istream& str, CSVRow& data) {
    data.readNextRow(str);
    return str;
  }

 public:
  typedef std::input_iterator_tag iterator_category;
  typedef CSVRow value_type;
  typedef std::size_t difference_type;
  typedef CSVRow* pointer;
  typedef CSVRow& reference;

  CSVIterator();
  CSVIterator(std::istream& str);

  // Pre Increment
  CSVIterator& operator++();

  // Post increment
  CSVIterator operator++(int);
  CSVRow const& operator*() const;
  CSVRow const* operator->() const;

  bool operator==(CSVIterator const& rhs);
  bool operator!=(CSVIterator const& rhs);

 private:
  std::istream* m_str;
  CSVRow m_row;
};

}  // namespace kimera
