require "arrow"

module CrySim
  # Feather/Parquet export via num.cr's `arrow` dependency (eltony81/arrow.cr).
  # Only available under `-Darrow` — this file is only required under that
  # flag (see crysim.cr), matching num.cr's own convention for its Arrow
  # backend. Every logged signal becomes a Float64 column; the time vector
  # is always the first column, named "t".
  class SimResult
    private def arrow_schema : Arrow::Schema
      fields = [Arrow::Field.new("t", Arrow::DataType.double)]
      fields.concat(@signals.keys.map { |k| Arrow::Field.new(k, Arrow::DataType.double) })
      Arrow::Schema.new(fields)
    end

    private def arrow_table(schema : Arrow::Schema) : Arrow::Table
      columns = [Arrow::DoubleArray.new(@t)] of Arrow::Array
      @signals.keys.each { |k| columns << Arrow::DoubleArray.new(@signals[k]) }
      Arrow::Table.new(schema, columns)
    end

    def to_feather(path : String)
      writer = Arrow::FeatherWriter.new(path)
      writer.write(arrow_table(arrow_schema))
      writer.close
    end

    def to_parquet(path : String)
      schema = arrow_schema
      writer = Arrow::ParquetWriter.new(schema, path)
      writer.write(arrow_table(schema))
      writer.close
    end
  end
end
