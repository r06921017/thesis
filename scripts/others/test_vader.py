#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from vaderSentiment.vaderSentiment import SentimentIntensityAnalyzer

analyser = SentimentIntensityAnalyzer()


def sentiment_analyzer_scores(sentence):
    score = analyser.polarity_scores(sentence)  # dictionary {'neg': 0.43, 'neu': 0.57, 'pos': 0.0, 'compound': -0.3}
    print("{:-<40} {}".format(sentence, str(score)))
    return


if __name__ == '__main__':
    s = 'Chat with me.'
    sentiment_analyzer_scores(s)
