/**
 * @file   csv_iterator.cpp
 * @brief  Parser for CSV files
 * @author Antoni Rosinol
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "kimera_semantics/csv_iterator.h"

namespace kimera {

std::string const& CSVIterator::CSVRow::operator[](std::size_t index) const {
  return m_data[index];
}

std::size_t CSVIterator::CSVRow::size() const { return m_data.size(); }

void CSVIterator::CSVRow::readNextRow(std::istream& str) {
  std::string line;
  std::getline(str, line);

  std::stringstream lineStream(line);
  std::string cell;

  m_data.clear();
  while (std::getline(lineStream, cell, ',')) {
    m_data.push_back(cell);
  }
  // This checks for a trailing comma with no data after it.
  if (!lineStream && cell.empty()) {
    // If there was a trailing comma then add an empty element.
    m_data.push_back("");
  }
}

CSVIterator::CSVIterator(std::istream& str) : m_str(str.good() ? &str : NULL) {
  ++(*this);
}

CSVIterator::CSVIterator() : m_str(NULL) {}

// Pre Increment
CSVIterator& CSVIterator::operator++() {
  if (m_str) {
    if (!((*m_str) >> m_row)) {
      m_str = NULL;
    }
  }
  return *this;
}

// Post increment
CSVIterator CSVIterator::operator++(int) {
  CSVIterator tmp(*this);
  ++(*this);
  return tmp;
}
CSVIterator::CSVRow const& CSVIterator::operator*() const { return m_row; }

CSVIterator::CSVRow const* CSVIterator::operator->() const { return &m_row; }

bool CSVIterator::operator==(CSVIterator const& rhs) {
  return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
}

bool CSVIterator::operator!=(CSVIterator const& rhs) {
  return !((*this) == rhs);
}

}  // namespace kimera
