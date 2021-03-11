#pragma once

class MockTimer {
public:
  MOCK_METHOD0(GetSeconds, double(void));
};
